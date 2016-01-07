#include "ensenso.hpp"
#include <dr_log/dr_log.hpp>
#include <dr_util/util.hpp>

#include <fstream>

namespace dr {

namespace {

void executeNxCommand(NxLibCommand & command) {
	try {
		command.execute();
	} catch (NxLibException const & e) {
		if (e.getErrorCode() == 17) {
			int code = command.result()[itmErrorSymbol].asInt();
			std::string message = command.result()[itmErrorText].asString();
			throw std::runtime_error("Failed to execute NxLibCommand: code " + std::to_string(code) + ": " + message);
		} else {
			throw std::runtime_error("NxLibException at " + e.getItemPath() + ": " + std::to_string(e.getErrorCode()) + ": " + e.getErrorText());
		}
	}
}

void executeNxCommand(NxLibCommand && command) {
	return executeNxCommand(command);
}


}

Ensenso::Ensenso(bool connect_overlay): found_overlay(false) {
	// initialize nxLib
	nxLibInitialize();

	// connect camera's
	NxLibItem cams = root[itmCameras][itmBySerialNo];

	// create an object referencing the camera's tree item, for easier access:
	for (int n = 0; n < cams.count(); n++) {
		if (cams[n][itmType] == valStereo)
			ensenso_camera = cams[n];
		else if (cams[n][itmType] == valMonocular && connect_overlay) {
			overlay_camera = cams[n];
			found_overlay = true;

			std::string serial = overlay_camera[itmSerialNumber].asString();
			NxLibCommand open(cmdOpen);
			open.parameters()[itmCameras] = serial;
			executeNxCommand(open);
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
	executeNxCommand(open);
}

Ensenso::~Ensenso() {
	executeNxCommand(NxLibCommand(cmdClose));
	nxLibFinalize();
}

cv::Size Ensenso::getIntensitySize() {
	int width, height;

	if (found_overlay) {
		overlay_camera[itmImages][itmRaw].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	} else {
		ensenso_camera[itmImages][itmRaw][itmLeft].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	}

	return cv::Size(width, height);
}

cv::Size Ensenso::getPointCloudSize() {
	int width, height;
	if (region_of_interest.area()) {
		width  = region_of_interest.width;
		height = region_of_interest.height;
	} else {
		ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	}

	return cv::Size(width, height);
}

void Ensenso::loadIntensity(cv::Mat & intensity) {
	// create data container
	cv::Size intensity_size = getIntensitySize();

	// no size
	if (intensity_size.area() == 0) {
		intensity = cv::Mat();
		return;
	}

	std::vector<uint8_t> data;
	if (found_overlay) {
		data.resize(intensity_size.area() * 3); // RGB
	} else {
		data.resize(intensity_size.area());
	}

	// capture images
	executeNxCommand(NxLibCommand(cmdCapture));

	// get binary data
	if (found_overlay) {
		overlay_camera[itmImages][itmRaw].getBinaryData(data, 0);
	} else {
		ensenso_camera[itmImages][itmRaw][itmLeft].getBinaryData(data, 0);
	}

	// copy to cv::Mat
	if (found_overlay) {
		intensity = cv::Mat(intensity_size, CV_8UC3);
		std::memcpy(intensity.data, data.data(), intensity_size.area() * sizeof(uint8_t) * 3);
		cv::cvtColor(intensity, intensity, cv::COLOR_RGB2BGR);
	} else {
		intensity = cv::Mat(intensity_size, CV_8UC1);
		std::memcpy(intensity.data, data.data(), intensity_size.area() * sizeof(uint8_t));
		cv::cvtColor(intensity, intensity, cv::COLOR_GRAY2BGR);
	}
}

void Ensenso::loadParameters(std::string const parameters_file) {
	std::ifstream file(parameters_file);
	std::stringstream buffer;
	buffer << file.rdbuf();

	int error;
	ensenso_camera[itmParameters].setJson(&error, buffer.str(), true);
	if (error != NxLibOperationSucceeded) {
		DR_ERROR("Could not set camera parameters. Error code: " << error);
	}
}

void Ensenso::loadPointCloud(
	PointCloudCamera::PointCloud & cloud,
	cv::Rect
) {
	try {
		// Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
		std::cout << "Grabbing an image" << std::endl;
		executeNxCommand(NxLibCommand(cmdCapture)); // Without parameters, most commands just operate on all open cameras
		std::cout << "Computing the disparity map" << std::endl; // This is the actual, computation intensive stereo matching task
		executeNxCommand(NxLibCommand(cmdComputeDisparityMap));
		std::cout << "Generating point map from disparity map" << std::endl; // This converts the disparity map into XYZ data for each pixel
		executeNxCommand(NxLibCommand(cmdComputePointMap));
		std::cout << "Done" << std::endl; // This converts the disparity map into XYZ data for each pixel

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
			executeNxCommand(render_pointmap);

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
		throw std::runtime_error("NxLibException at " + e.getItemPath() + ": " + std::to_string(e.getErrorCode()) + ": " + e.getErrorText());
	}
}

void Ensenso::setRegionOfInterest(cv::Rect const & roi) {
	ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0]     = roi.tl().x;
	ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1]     = roi.tl().y;
	ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0] = roi.br().x;
	ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1] = roi.br().y;

	region_of_interest = roi;
}

}
