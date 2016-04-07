#pragma once
#include "util.hpp"

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ensenso/nxLib.h>
#include <boost/optional.hpp>

namespace dr {

class Ensenso {
protected:
	/// The root EnsensoSDK node.
	NxLibItem root;

	/// The Ensenso camera node.
	NxLibItem ensenso_camera;

	/// The overlay camera node.
	boost::optional<NxLibItem> overlay_camera;

public:
	/// Connect to an ensenso camera.
	Ensenso(bool connect_overlay = true);

	/// Destructor.
	~Ensenso();

	/// Get the native nxLibItem for the stereo camera.
	NxLibItem native() const {
		return ensenso_camera;
	}

	/// Get the native nxLibItem for the overlay camera (if any).
	boost::optional<NxLibItem> nativeOverlay() const {
		return overlay_camera;
	}

	/// Get the serial number of the stereo camera.
	std::string serialNumber() const {
		return getNx<std::string>(ensenso_camera[itmSerialNumber]);
	}

	/// Get the serial number of the overlay camera or an empty string if there is no overlay camera.
	std::string overlaySerialNumber() const {
		return overlay_camera ? getNx<std::string>(overlay_camera.get()[itmSerialNumber]) : "";
	}

	/// Loads the camera parameters from a JSON file.
	void loadParameters(std::string const parameters_file);

	/// Loads the overlay camera parameters from a JSON file.
	void loadOverlayParameters(std::string const parameters_file);

	/// Loads the overlay camera uEye parameters from a INI file.
	void loadOverlayParameterSet(std::string const parameters_file);

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
	bool calibrate(int const num_patterns, Eigen::Isometry3d & pose) const;

	/// Returns the size of the intensity images.
	cv::Size getIntensitySize();

	/// Returns the size of the depth images.
	cv::Size getPointCloudSize();

	/// Loads the intensity image to intensity.
	/**
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadIntensity(cv::Mat & intensity, bool capture);

	/// Loads the intensity image to intensity.
	void loadIntensity(cv::Mat & intensity) {
		loadIntensity(intensity, true);
	}

	/// Loads the pointcloud from depth in the region of interest.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi, bool capture);

	/// Loads the pointcloud from depth in the region of interest.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 */
	void loadPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi = cv::Rect()) {
		return loadPointCloud(cloud, roi, true);
	}

	/// Get a pointlcoud from the camera.
	/**
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	pcl::PointCloud<pcl::PointXYZ> getPointCloud(cv::Rect roi, bool capture) {
		pcl::PointCloud<pcl::PointXYZ> result;
		loadPointCloud(result, roi, capture);
		return result;
	}

	/// Loads the pointcloud registered to the overlay camera.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadRegisteredPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi, bool capture = true);

protected:
	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	void setRegionOfInterest(cv::Rect const & roi);

};

}
