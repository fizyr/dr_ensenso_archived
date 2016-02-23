#pragma once

#include <dr_camera/intensity_camera.hpp>
#include <dr_camera/depth_camera.hpp>
#include <dr_camera/point_cloud_camera.hpp>
#include <dr_camera_parameters/intrinsic_parameters.hpp>

#include <ensenso/nxLib.h>

namespace dr {

class Ensenso : public IntensityCamera, public PointCloudCamera {
public:
	Ensenso(bool connect_overlay = true);

	~Ensenso();

	/// Get new data.
	void capture();

	/// Returns the size of the intensity images.
	cv::Size getIntensitySize() override;

	/// Returns the size of the depth images.
	cv::Size getPointCloudSize() override;

	/// Loads the intensity image to intensity.
	void loadIntensity(cv::Mat & intensity) override;
	void loadIntensity(NxLibItem const & item, cv::Mat & intensity) const;

	/// Loads the (stereo) images from the Ensenso.
	void loadLeftIntensity(cv::Mat & intensity) const;
	void loadRightIntensity(cv::Mat & intensity) const;

	cv::Mat getLeftIntensity() const;
	cv::Mat getRightIntensity() const;

	/// Get intrinsic parameters for Ensenso.
	IntrinsicParameters getIntrinsics(NxLibItem const & item) const;
	IntrinsicParameters getLeftIntrinsics() const;
	IntrinsicParameters getRightIntrinsics() const;

	Eigen::Matrix4d getReprojectionMatrix() const;

	/// Loads the camera parameters from a JSON file.
	void loadParameters(std::string const parameters_file);

	/**
	 * Loads the pointcloud from depth in the region of interest.
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 */
	void loadPointCloud(PointCloudCamera::PointCloud & cloud, cv::Rect roi = cv::Rect()) override;

	/// Turns the front light on or off.
	void setFrontLight(bool state);

	/// Turns the projector on or off.
	void setProjector(bool state);

protected:
	/// The root EnsensoSDK node.
	NxLibItem root;

	/// The Ensenso camera node.
	NxLibItem ensenso_camera;

	/// True if a connected overlay camera is found.
	bool found_overlay;

	/// The overlay camera node.
	NxLibItem overlay_camera;

	/// Conversion from ensenso timestamp to PCL timestamp.
	inline pcl::uint64_t ensensoStampToPcl(double stamp) { return (stamp - 11644473600.0) * 1000000.0; };

	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	void setRegionOfInterest(cv::Rect const & roi);

};

}
