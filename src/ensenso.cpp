#include "ensenso.hpp"
#include <dr_uv_mapping/uv_mapping.hpp>
#include <dr_util/util.hpp>

namespace dr {

Ensenso::Ensenso() :
	found_overlay(false)
{
	//std::cout << "Opening NxLib and waiting for cameras to be detected." << std::endl;

	// initialize nxLib
	nxLibInitialize();

	// connect camera's
	NxLibItem cams = root[itmCameras][itmBySerialNo];

	// Print information for all cameras in the tree
	//std::cout << "Number of connected cameras: " << cams.count() << std::endl;
	//std::cout << "Serial No    Model   Status" << std::endl;

	//for (int n = 0; n < cams.count(); ++n) {
	//	std::cout << cams[n][itmSerialNumber].asString()
	//		<< " " << cams[n][itmModelName].asString()
	//		<< " " << cams[n][itmStatus].asString() << "\n";
	//}
	//std::cout << std::endl;

	// create an object referencing the camera's tree item, for easier access:
	for (int n = 0; n < cams.count(); n++) {
		if (cams[n][itmType] == valStereo)
			ensenso_camera = cams[n];
		else if (cams[n][itmType] == valMonocular) {
			//std::cout << "Opening overlay camera..." << std::endl;

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
	//std::cout << "Opening camera " << serial << std::endl;
	NxLibCommand open(cmdOpen);
	open.parameters()[itmCameras] = serial;
	open.execute();
}

Ensenso::~Ensenso() {
	//std::cout << "Closing camera" << std::endl;
	NxLibCommand(cmdClose).execute();
	nxLibFinalize();
}

cv::Size Ensenso::getIntensitySize() {
	try {
		int width;
		int height;
		if (found_overlay) {
			overlay_camera[itmImages][itmRaw].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
		} else {
			ensenso_camera[itmImages][itmRaw][itmLeft].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
		}
		return cv::Size(width, height);
	} catch (NxLibException & e) {
		//std::cout << "An NxLib API error with code " << e.getErrorCode() << " " << e.getErrorText() << " occurred while accessing item " << e.getItemPath() << std::endl;
	}

	return cv::Size();
}

cv::Size Ensenso::getDepthSize() {
	try {
		int width;
		int height;
		ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
		return cv::Size(width, height);
	} catch (NxLibException & e) {
		//std::cout << "An NxLib API error with code " << e.getErrorCode() << " " << e.getErrorText() << " occurred while accessing item " << e.getItemPath() << std::endl;
	}

	return cv::Size();
}

void Ensenso::loadIntensity(cv::Mat & intensity) {
	try {
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
	} catch (NxLibException & e) {
		//std::cout << "An NxLib API error with code " << e.getErrorCode() << " " << e.getErrorText() << " occurred while accessing item " << e.getItemPath() << std::endl;
	}
}

void Ensenso::loadDepth(cv::Mat & depth) {
	try {
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
	} catch (NxLibException & e) {
		//std::cout << "An NxLib API error with code " << e.getErrorCode() << " " << e.getErrorText() << " occurred while accessing item " << e.getItemPath() << std::endl;
	}
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
	try {
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
		cloud.header.stamp = ensensoStampToPcl(timestamp);
		cloud.width        = width;
		cloud.height       = height;
		cloud.is_dense     = false;
		cloud.resize(height * width);

		// Copy data in point cloud (and convert milimeters in meters)
		for (size_t i = 0; i < point_map.size (); i += 3) {
			cloud.points[i / 3].x = point_map[i] / 1000.0;
			cloud.points[i / 3].y = point_map[i + 1] / 1000.0;
			cloud.points[i / 3].z = point_map[i + 2] / 1000.0;
		}
	} catch (NxLibException & e) {
		//std::cout << "An NxLib API error with code " << e.getErrorCode() << " " << e.getErrorText() << " occurred while accessing item " << e.getItemPath() << std::endl;
	}
}

pcl::PointXYZ Ensenso::get3d(cv::Point const &, double) {
	throw std::runtime_error("Not supported.");
	return pcl::PointXYZ();
}


}
