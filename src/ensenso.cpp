#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include "pcl.hpp"

#include <dr_log/dr_log.hpp>
#include <dr_util/util.hpp>

#include <stdexcept>

namespace dr {

Ensenso::Ensenso(bool connect_overlay) {
	// Initialize nxLib.
	nxLibInitialize();

	// Try to find a stereo camera.
	boost::optional<NxLibItem> camera = openCameraByType(valStereo);
	if (!camera) throw std::runtime_error("Please connect a single stereo camera to your computer.");
	ensenso_camera = *camera;

	// Get the linked overlay camera.
	if (connect_overlay) overlay_camera = openCameraByLink(serialNumber());
}

Ensenso::~Ensenso() {
	executeNx(NxLibCommand(cmdClose));
	nxLibFinalize();
}

void Ensenso::loadParameters(std::string const parameters_file) {
	setNxJsonFromFile(ensenso_camera[itmParameters], parameters_file);
}

void Ensenso::loadOverlayParameters(std::string const parameters_file) {
	if (!overlay_camera) throw std::runtime_error("No overlay camera found. Can not load overlay parameters.");
	setNxJsonFromFile(overlay_camera.get()[itmParameters], parameters_file);
}

bool Ensenso::trigger(bool stereo, bool overlay) const {
	overlay = overlay && overlay_camera;

	NxLibCommand command(cmdTrigger);
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (overlay) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(overlay_camera.get()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmTriggered])) return false;
	if (overlay && !getNx<bool>(command.result()[overlaySerialNumber()][itmTriggered])) return false;
	return true;
}

bool Ensenso::retrieve(bool trigger, unsigned int timeout, bool stereo, bool overlay) const {
	overlay = overlay && overlay_camera;

	NxLibCommand command(trigger ? cmdCapture : cmdRetrieve);
	setNx(command.parameters()[itmTimeout], int(timeout));
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (overlay) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(overlay_camera.get()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmRetrieved])) return false;
	if (overlay && !getNx<bool>(command.result()[overlaySerialNumber()][itmRetrieved])) return false;
	return true;
}

void Ensenso::calibrate(int const num_patterns, Eigen::Isometry3d & pose) const {
	executeNx(NxLibCommand(cmdDiscardPatterns));

	for (int i = 0; i < num_patterns; ++i) {
		// Capture image with front-light.
		setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], false);
		setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], true);

		retrieve(true, 1500, true, false);
		NxLibCommand command_capture(cmdCapture);
		executeNx(command_capture);

		setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], false);
		setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], true);

		// Find the pattern.
		NxLibCommand command_collect_pattern(cmdCollectPattern);
		setNx(command_collect_pattern.parameters()[itmCameras], serialNumber());
		setNx(command_collect_pattern.parameters()[itmDecodeData], true);
		executeNx(command_collect_pattern);
	}

	// Get the pose of the pattern.
	NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
	executeNx(command_estimate_pose);
	pose = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
	pose.translation() *= 0.001;
}

cv::Size Ensenso::getIntensitySize() {
	int width, height;

	try {
		overlay_camera.get()[itmImages][itmRaw].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
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

void Ensenso::loadIntensity(cv::Mat & intensity, bool capture) {
	if (capture) this->retrieve(true, 1500, !overlay_camera, !!overlay_camera);

	// Copy to cv::Mat.
	if (overlay_camera) {
		cv::cvtColor(toCvMat(overlay_camera.get()[itmImages][itmRaw]), intensity, cv::COLOR_RGB2BGR);
	} else {
		cv::cvtColor(toCvMat(ensenso_camera[itmImages][itmRaw][itmLeft]), intensity, cv::COLOR_GRAY2BGR);
	}
}

void Ensenso::loadPointCloud(PointCloudCamera::PointCloud & cloud, cv::Rect roi, bool capture) {
	// Optionally capture new data.
	if (capture) this->retrieve();

	setRegionOfInterest(roi);
	std::string serial = serialNumber();

	// Compute disparity.
	{
		NxLibCommand command(cmdComputeDisparityMap);
		setNx(command.parameters()[itmCameras], serial);
		executeNx(command);
	}

	// Compute point cloud.
	{
		NxLibCommand command(cmdComputePointMap);
		setNx(command.parameters()[itmCameras], serial);
		executeNx(command);
	}

	// Convert the binary data to a point cloud.
	cloud = toPointCloud(ensenso_camera[itmImages][itmPointMap]);
}

void Ensenso::setRegionOfInterest(cv::Rect const & roi) {
	if (roi.area() == 0) {
		setNx(ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest], false);

		if (ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].exists()) {
			ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].erase();
		}
	} else {
		setNx(ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest],          true);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0],     roi.tl().x);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1],     roi.tl().y);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0], roi.br().x);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1], roi.br().y);
	}

}

}
