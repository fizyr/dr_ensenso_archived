#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"

#include <dr_log/dr_log.hpp>
#include <dr_util/util.hpp>

#include <fstream>

namespace dr {

Ensenso::Ensenso(bool connect_overlay): found_overlay(false) {
	// initialize nxLib
	nxLibInitialize();

	// connect camera's
	NxLibItem cams = root[itmCameras][itmBySerialNo];

	// create an object referencing the camera's tree item, for easier access:
	for (int n = 0; n < cams.count(); n++) {
		if (cams[n][itmType] == valStereo) {
			ensenso_camera = cams[n];
		} else if (cams[n][itmType] == valMonocular && connect_overlay) {
			overlay_camera = cams[n];
			found_overlay = true;

			std::string serial = overlay_camera[itmSerialNumber].asString();
			NxLibCommand open(cmdOpen);
			open.parameters()[itmCameras] = serial;
			executeNx(open);
		}
	}

	// found camera?
	if (!ensenso_camera.exists() || (ensenso_camera[itmType] != valStereo)) {
		throw std::runtime_error("Please connect a single stereo camera to your computer.");
	}

	// open camera
	std::string serial = ensenso_camera[itmSerialNumber].asString();
	NxLibCommand open(cmdOpen);
	open.parameters()[itmCameras] = serial;
	executeNx(open);
}

Ensenso::~Ensenso() {
	executeNx(NxLibCommand(cmdClose));
	nxLibFinalize();
}

void Ensenso::capture() {
	executeNx(NxLibCommand(cmdCapture));
}

cv::Size Ensenso::getIntensitySize() {
	int width, height;

	try {
		overlay_camera[itmImages][itmRaw].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	} catch (NxLibException const & e) {
		throw NxError(e);
	}

	return cv::Size(width, height);
}

cv::Size Ensenso::getPointCloudSize() {
	int width, height;
	ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	return cv::Size(width, height);
}

Eigen::Matrix4d Ensenso::getReprojectionMatrix() const {
	Eigen::Matrix4d reprojection =
		toEigenMatrix<4, 4>(ensenso_camera[itmCalibration][itmStereo][itmReprojection]);

	// Adjust the reprojection matrix to work with meters.
	reprojection.block(0, 0, 3, 4) *= 0.001;

	return reprojection;
}

IntrinsicParameters Ensenso::getIntrinsics(NxLibItem const & item) const {
	IntrinsicParameters intrinsics;

	double fx = getNx<double>(item[itmCamera][0][0]);
	double fy = getNx<double>(item[itmCamera][1][1]);
	double cx = getNx<double>(item[itmCamera][2][0]);
	double cy = getNx<double>(item[itmCamera][2][1]);

	double k1 = getNx<double>(item[itmDistortion][0]);
	double k2 = getNx<double>(item[itmDistortion][1]);
	double p1 = getNx<double>(item[itmDistortion][2]);
	double p2 = getNx<double>(item[itmDistortion][3]);
	double k3 = getNx<double>(item[itmDistortion][4]);

	intrinsics.setFocalLength(cv::Point2d(fx, fy));
	intrinsics.setImageCenter(cv::Point2d(cx, cy));
	intrinsics.setDistortionParameters((cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3));

	return intrinsics;
}

IntrinsicParameters Ensenso::getLeftIntrinsics() const {
	return getIntrinsics(ensenso_camera[itmCalibration][itmDynamic][itmStereo][itmLeft]);
}

IntrinsicParameters Ensenso::getRightIntrinsics() const {
	return getIntrinsics(ensenso_camera[itmCalibration][itmDynamic][itmStereo][itmRight]);
}

void Ensenso::loadIntensity(NxLibItem const & item, cv::Mat & intensity) const {
	intensity = toCvMat(item).clone();
}

void Ensenso::loadIntensity(cv::Mat & intensity) {
	loadIntensity(overlay_camera[itmImages][itmRaw], intensity);
	cv::cvtColor(intensity, intensity, cv::COLOR_RGB2BGR);
}

void Ensenso::setFrontLight(bool state) {
	setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], state);
}

void Ensenso::setProjector(bool state) {
	setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], state);
}

void Ensenso::loadLeftIntensity(cv::Mat & intensity) const {
	loadIntensity(ensenso_camera[itmImages][itmRectified][itmLeft], intensity);
}

void Ensenso::loadRightIntensity(cv::Mat & intensity) const {
	loadIntensity(ensenso_camera[itmImages][itmRectified][itmRight], intensity);
}


cv::Mat Ensenso::getLeftIntensity() const {
	cv::Mat intensity;
	loadLeftIntensity(intensity);
	return intensity;
}

cv::Mat Ensenso::getRightIntensity() const {
	cv::Mat intensity;
	loadRightIntensity(intensity);
	return intensity;

}

void Ensenso::loadParameters(std::string const parameters_file) {
	std::ifstream file(parameters_file);
	std::stringstream buffer;
	buffer << file.rdbuf();

	int error = 0;
	ensenso_camera[itmParameters].setJson(&error, buffer.str(), true);
	if (error) throw NxError(ensenso_camera[itmParameters], error);
}

void Ensenso::loadPointCloud(PointCloudCamera::PointCloud & cloud, cv::Rect roi) {
	try {
		setRegionOfInterest(roi);

		// Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
		NxLibCommand capture(cmdCapture);
		capture.parameters()[itmTimeout] = 1500;
		executeNx(capture); // Capture new data.
		executeNx(NxLibCommand(cmdComputeDisparityMap));
		executeNx(NxLibCommand(cmdComputePointMap));

		// Get info about the computed point map and copy it into a std::vector
		double timestamp;
		std::vector<float> point_map;
		int width;
		int height;

		if (found_overlay) {
			NxLibCommand render_pointmap(cmdRenderPointMap);
			render_pointmap.parameters()[itmCamera]            = overlay_camera[itmSerialNumber].asString();
			render_pointmap.parameters()[itmCameras]           = ensenso_camera[itmSerialNumber].asString();
			render_pointmap.parameters()[itmNear]              = 50; // must be set
			executeNx(render_pointmap);

			root[itmImages][itmRenderPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
			root[itmImages][itmRenderPointMap].getBinaryData(point_map, 0);
		} else {
			ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
			ensenso_camera[itmImages][itmPointMap].getBinaryData(point_map, 0);
		}

		// Copy point cloud and convert in meters
		cloud.header.stamp    = ensensoStampToPcl(timestamp);
		cloud.header.frame_id = "/camera_link";
		cloud.width           = width;
		cloud.height          = height;
		cloud.is_dense        = false;
		cloud.resize(height * width);

		// Copy data in point cloud (and convert milimeters in meters)
		for (size_t i = 0; i < point_map.size (); i += 3) {
			cloud.points[i / 3].x = point_map[i] / 1000.0;
			cloud.points[i / 3].y = point_map[i + 1] / 1000.0;
			cloud.points[i / 3].z = point_map[i + 2] / 1000.0;
		}
	} catch (NxLibException const & e) {
		throw NxError(e);
	}
}

void Ensenso::setRegionOfInterest(cv::Rect const & roi) {
	if (roi.area() == 0) {
		ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest] = false;

		if (ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].exists()) {
			ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].erase();
		}
	} else {
		ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest]          = true;
		ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0]     = roi.tl().x;
		ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1]     = roi.tl().y;
		ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0] = roi.br().x;
		ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1] = roi.br().y;
	}

}

}
