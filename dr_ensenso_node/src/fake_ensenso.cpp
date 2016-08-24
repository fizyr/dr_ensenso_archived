#include <ros/ros.h>
#include <dr_ensenso_msgs/GetCameraData.h>

#include <dr_param/param.hpp>
#include <dr_ros/node.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

namespace dr {

class FakeCameraNode : public Node {
private:
	/// Service server for supplying point clouds and images.
	ros::ServiceServer get_data_server;

	/// Buffered image.
	cv::Mat image;

	/// Buffered point cloud.
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	/// Name for the camera frame.
	std::string camera_frame;

public:
	FakeCameraNode() {
		camera_frame    = getParam<std::string>("camera_frame");
		get_data_server = advertiseService("get_data" , &FakeCameraNode::getData, this);
		DR_SUCCESS("Node initialized.");
	}

private:
	bool getData(dr_ensenso_msgs::GetCameraData::Request &, dr_ensenso_msgs::GetCameraData::Response & res) {
		// read image file path
		std::string image_file = getParam<std::string>("image");
		std::string point_cloud_file = getParam<std::string>("point_cloud");

		if (!boost::filesystem::exists(image_file))       DR_ERROR("Failed to load image: File does not exist: " << image_file);
		if (!boost::filesystem::exists(point_cloud_file)) DR_ERROR("Failed to load point cloud: File does not exist: " << point_cloud_file);

		// load image
		image = cv::imread(image_file);
		if (image.empty()) {
			DR_ERROR("Failed to load image from path: " << image_file);
			return false;
		}

		// load point cloud
		if (pcl::io::loadPCDFile(point_cloud_file, point_cloud) == -1 || point_cloud.empty()) {
			DR_ERROR("Failed to load point cloud from path: " << point_cloud_file);
			return false;
		}

		// copy the image
		std_msgs::Header header;
		header.frame_id = camera_frame;
		header.stamp = ros::Time::now();
		cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image);
		res.color = *cv_image.toImageMsg();

		// copy the point cloud
		pcl::toROSMsg(point_cloud, res.point_cloud);
		res.point_cloud.header = header;
		return true;
	}

};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_fake_camera");
	dr::FakeCameraNode node;
	ros::spin();
}
