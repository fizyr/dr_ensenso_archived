#pragma once
#include "util.hpp"

#include <dr_camera/intensity_camera.hpp>
#include <dr_camera/depth_camera.hpp>
#include <dr_camera/point_cloud_camera.hpp>
#include <dr_camera_parameters/intrinsic_parameters.hpp>

#include <ensenso/nxLib.h>
#include <boost/optional.hpp>

namespace dr {

class Ensenso : public IntensityCamera, public PointCloudCamera {
public:
	Ensenso(bool connect_overlay = true);

	~Ensenso();

	/// Returns the size of the intensity images.
	cv::Size getIntensitySize() override;

	/// Returns the size of the depth images.
	cv::Size getPointCloudSize() override;

	/// Loads the intensity image to intensity.
	/**
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadIntensity(cv::Mat & intensity, bool capture);

	/// Loads the intensity image to intensity.
	void loadIntensity(cv::Mat & intensity) override {
		loadIntensity(intensity, true);
	}

	/// Loads the camera parameters from a JSON file.
	void loadParameters(std::string const parameters_file);

	/// Trigger data acquisition on the camera.
	/**
	 * \param stereo If true, capture data from the stereo camera.
	 * \param overlay If true, capture data from the overlay camera.
	 */
	bool trigger(bool stereo = true, bool overlay=true) const;

	/// Retrieve new data from the camera without sending a software trigger.
	/**
	 * \param timeout A timeout in milliseconds.
	 * \param stereo If true, capture data from the stereo camera.
	 * \param overlay If true, capture data from the overlay camera.
	 */
	bool retrieve(bool trigger = true, unsigned int timeout = 1500, bool stereo = true, bool overlay=true) const;

	/// Returns the pose of the camera with respect to the calibration plate.
	void calibrate(int const num_patterns, Eigen::Isometry3d & pose) const;

	/**
	 * Loads the pointcloud from depth in the region of interest.
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadPointCloud(PointCloudCamera::PointCloud & cloud, cv::Rect roi, bool capture);

	using PointCloudCamera::getPointCloud;

	pcl::PointCloud<pcl::PointXYZ> getPointCloud(cv::Rect roi, bool capture) {
		pcl::PointCloud<pcl::PointXYZ> result;
		loadPointCloud(result, roi, capture);
		return result;
	}

	/**
	 * Loads the pointcloud from depth in the region of interest.
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 */
	void loadPointCloud(PointCloudCamera::PointCloud & cloud, cv::Rect roi = cv::Rect()) override {
		return loadPointCloud(cloud, roi, true);
	}

	/// Get the native nxLibItem for the stereo camera.
	NxLibItem getNativeCamera() const {
		return ensenso_camera;
	}

	/// Get the native nxLibItem for the overlay camera (if any).
	boost::optional<NxLibItem> getNativeOverlayCamera() const {
		return found_overlay ? boost::optional<NxLibItem>{overlay_camera} : boost::none;
	}

	/// Get the serial number of the stereo camera.
	std::string getSerialNumber() const {
		return getNx<std::string>(ensenso_camera[itmSerialNumber]);
	}

	/// Get the serial number of the overlay camera or an empty string if there is no overlay camera.
	std::string getOverlaySerialNumber() const {
		return found_overlay ? getNx<std::string>(overlay_camera[itmSerialNumber]) : "";
	}

protected:
	/// The root EnsensoSDK node.
	NxLibItem root;

	/// The Ensenso camera node.
	NxLibItem ensenso_camera;

	/// True if a connected overlay camera is found.
	bool found_overlay;

	/// The overlay camera node.
	NxLibItem overlay_camera;

	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	void setRegionOfInterest(cv::Rect const & roi);

};

}
