#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include <pcl.hpp>

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

bool Ensenso::calibrate(int const num_patterns, Eigen::Isometry3d & pose) const {
	try {

		executeNx(NxLibCommand(cmdDiscardPatterns));

		for (int i = 0; i < num_patterns; ++i) {
			// Capture image with front-light.
			setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], false);
			setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], true);

			NxLibCommand command_capture(cmdCapture);
			setNx(command_capture.parameters()[itmCameras], ensenso_camera[itmEepromId].asInt());
			executeNx(command_capture);

			setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], false);
			setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], true);

			// Find the pattern.
			NxLibCommand command_collect_pattern(cmdCollectPattern);
			setNx(command_collect_pattern.parameters()[itmCameras], ensenso_camera[itmEepromId].asInt());
			setNx(command_collect_pattern.parameters()[itmDecodeData], true);
			executeNx(command_collect_pattern);
		}

		/// Get the pose of the pattern.
		NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
		executeNx(command_estimate_pose);
		pose = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
		pose.translation() *= 0.001;

	} catch (std::runtime_error const & e) {
		DR_INFO("An unexpected error occurred during calibration: " << e.what());
		return false;
	}

	return true;
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

void Ensenso::loadIntensity(cv::Mat & intensity) {
	// capture images
	executeNx(NxLibCommand(cmdCapture));

	// copy to cv::Mat
	if (found_overlay) {
		cv::cvtColor(toCvMat(overlay_camera[itmImages][itmRaw]), intensity, cv::COLOR_RGB2BGR);
	} else {
		cv::cvtColor(toCvMat(ensenso_camera[itmImages][itmRaw][itmLeft]), intensity, cv::COLOR_GRAY2BGR);
	}
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
	setRegionOfInterest(roi);

	// Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
	NxLibCommand capture(cmdCapture);
	setNx(capture.parameters()[itmTimeout], 1500);
	executeNx(capture); // Capture new data.
	executeNx(NxLibCommand(cmdComputeDisparityMap));
	executeNx(NxLibCommand(cmdComputePointMap));

	cloud = toPointCloud(ensenso_camera[itmImages][itmPointMap]);
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
