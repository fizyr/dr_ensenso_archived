#include "ensenso.hpp"
#include <dr_log/dr_log.hpp>
#include <dr_uv_mapping/uv_mapping.hpp>
#include <dr_util/util.hpp>

namespace dr {

Ensenso::Ensenso(): found_overlay(false) {
	// initialize nxLib
	nxLibInitialize();

	// connect camera's
	NxLibItem cams = root[itmCameras][itmBySerialNo];

	// create an object referencing the camera's tree item, for easier access:
	for (int n = 0; n < cams.count(); n++) {
		if (cams[n][itmType] == valStereo)
			ensenso_camera = cams[n];
		else if (cams[n][itmType] == valMonocular) {
			overlay_camera = cams[n];
			found_overlay = true;

			std::string serial = overlay_camera[itmSerialNumber].asString();
			NxLibCommand open(cmdOpen);
			open.parameters()[itmCameras] = serial;
			open.execute();
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
	open.execute();
}

Ensenso::~Ensenso() {
	NxLibCommand(cmdClose).execute();
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

cv::Size Ensenso::getDepthSize() {
	int width, height;
	ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);

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
	NxLibCommand(cmdCapture).execute();

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
	FILE * file = fopen(parameters_file.c_str(), "r");

	if (!file) {
		DR_ERROR("Could not load parameters file '" << parameters_file << "'.");
		return;
	}

	int num_char = 0, error = 0;
	int const max_length = 1024 * 64 + 1;
	char str[max_length];

	while (true) {
		int readchar = getc(file);
		if (EOF == readchar || max_length == num_char) {
			str[num_char] = '\0';
			break;
		}
		str[num_char++] = (char)readchar;
	}

	fclose(file);

	nxLibSetJson(&error, "Cameras/ByEepromId/\\1/Parameters", str, NXLIBTRUE);
	if (error != NxLibOperationSucceeded) {
		DR_ERROR("Could not set camera parameters. Error code: " << error);
	}
}

void Ensenso::loadDepth(cv::Mat & depth) {
	// create data container
	cv::Size depth_size = getDepthSize();
	// no size
	if (depth_size.area() == 0) {
		depth = cv::Mat();
		return;
	}

	std::vector<uint8_t> data(depth_size.area());

	// capture images
	NxLibCommand(cmdCapture).execute();

	// get binary data
	ensenso_camera[itmImages][itmRaw][itmRight].getBinaryData(data, 0);

	// copy to cv::Mat
	depth = cv::Mat(depth_size, CV_8UC1);
	std::memcpy(depth.data, data.data(), depth_size.area() * sizeof(uint8_t));
	// data is expected as uint16_t
	depth.convertTo(depth, CV_16UC1);
}

std::unique_ptr<UvMapping> Ensenso::getUvMapping() {
	// return "empty" UV mapping
	if (found_overlay) {
		return make_unique<LinearUvMapping>(getIntensitySize(), getIntensitySize());
	} else {
		return make_unique<LinearUvMapping>(getDepthSize(), getIntensitySize());
	}
}

void Ensenso::loadPointCloud(
	cv::Mat const &,
	DepthCamera::PointCloud & cloud,
	cv::Rect
) {
	// Execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
	//std::cout << "Grabbing an image" << std::endl;
	NxLibCommand(cmdCapture).execute(); // Without parameters, most commands just operate on all open cameras
	//std::cout << "Computing the disparity map" << std::endl; // This is the actual, computation intensive stereo matching task
	NxLibCommand(cmdComputeDisparityMap).execute();
	//std::cout << "Generating point map from disparity map" << std::endl; // This converts the disparity map into XYZ data for each pixel
	NxLibCommand(cmdComputePointMap).execute();

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
		render_pointmap.execute();

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
}

pcl::PointXYZ Ensenso::get3d(cv::Point const &, double) {
	throw std::runtime_error("Not supported.");
	return pcl::PointXYZ();
}


}
