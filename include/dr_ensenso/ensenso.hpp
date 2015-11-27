#pragma once

#include <dr_camera/intensity_camera.hpp>
#include <dr_camera/depth_camera.hpp>
#include <dr_camera/uv_mapping_source.hpp>

#include <nxLib.h>

namespace dr {

class Ensenso : public IntensityCamera, public DepthCamera, public UvMappingSource {
public:
	Ensenso();

	~Ensenso();

	/// Returns the size of the intensity images.
	cv::Size getIntensitySize() override;

	/// Returns the size of the depth images.
	cv::Size getDepthSize() override;

	/// Get the UV mapping for this camera.
	std::unique_ptr<UvMapping> getUvMapping() override;

	/// Loads the intensity image to intensity.
	void loadIntensity(cv::Mat & intensity) override;

	/// Loads the depth image to depth.
	void loadDepth(cv::Mat & depth) override;

	/// Loads the camera parameters from a JSON file.
	void loadParameters(std::string const parameters_file);

	/**
	 * Loads the pointcloud from depth in the region of interest.
	 * \param depth The depth image.
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 */
	void loadPointCloud(
		cv::Mat const & depth,
		DepthCamera::PointCloud & cloud,
		cv::Rect roi = cv::Rect()
	) override;

	/// Translates a single depth point to a XYZ point
	pcl::PointXYZ get3d(cv::Point const &, double) override;

	/// Returns whether there is a new depth image available.
	inline bool hasNewDepth() override { return true; }

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
};

}
