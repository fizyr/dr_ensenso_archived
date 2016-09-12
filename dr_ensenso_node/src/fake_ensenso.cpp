#include <ros/ros.h>
#include <dr_ensenso_msgs/Calibrate.h>
#include <dr_ensenso_msgs/FinalizeCalibration.h>
#include <dr_ensenso_msgs/InitializeCalibration.h>
#include <dr_ensenso_msgs/GetCameraData.h>
#include <dr_ensenso_msgs/GetPatternPose.h>
#include <dr_msgs/SendPose.h>
#include <dr_msgs/SendPoseStamped.h>

#include <dr_param/param.hpp>
#include <dr_ros/node.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

namespace dr {

class FakeEnsensoNode : public Node {
private:
	/// Service server for supplying point clouds and images.
	ros::ServiceServer get_data_server;

	/// Buffered image.
	cv::Mat image;

	/// Buffered point cloud.
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	/// Name for the camera frame.
	std::string camera_frame;

	/// If true, publishes point cloud data when calling getData.
	bool publish_cloud = true;

	/// If true, publishes point cloud data when calling getData.
	bool publish_image = true;

	/// Object for transporting images.
	image_transport::ImageTransport image_transport;

	struct {
		/// Service server for supplying point clouds and images.
		ros::ServiceServer camera_data;

		/// Service server for dumping image and cloud to disk.
		ros::ServiceServer dump_data;

		/// Service server for retrieving the pose of the pattern.
		ros::ServiceServer get_pattern_pose;

		/// Service server for initializing the calibration sequence.
		ros::ServiceServer initialize_calibration;

		/// Service server for recording one calibration sample.
		ros::ServiceServer record_calibration;

		/// Service server for finalizing the calibration.
		ros::ServiceServer finalize_calibration;

		/// Service server for setting the camera pose setting of the Ensenso.
		ros::ServiceServer set_workspace_calibration;

		/// Service server for clearing the camera pose setting of the Ensenso.
		ros::ServiceServer clear_workspace_calibration;

		/// Service server combining 'get_pattern_pose', 'set_workspace', and stores it to the ensenso.
		ros::ServiceServer calibrate_workspace;

		/// Service server for storing the calibration.
		ros::ServiceServer store_workspace_calibration;
	} servers;

	struct {
		ros::Publisher cloud;
		image_transport::Publisher image;
	} publishers;

public:
	FakeEnsensoNode() : image_transport(*this) {
		camera_frame                   = getParam<std::string>("camera_frame");
		publish_cloud                  = getParam<bool>("publish_cloud", publish_cloud, true);
		publish_image                  = getParam<bool>("publish_image", publish_image, true);

		publishers.cloud               = advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
		publishers.image               = image_transport.advertise("image", 1, true);

		servers.camera_data                 = advertiseService("get_data"                   , &FakeEnsensoNode::onGetData                   , this);
		servers.dump_data                   = advertiseService("dump_data"                  , &FakeEnsensoNode::onDumpData                  , this);
		servers.get_pattern_pose            = advertiseService("get_pattern_pose"           , &FakeEnsensoNode::onGetPatternPose            , this);
		servers.initialize_calibration      = advertiseService("initialize_calibration"     , &FakeEnsensoNode::onInitializeCalibration     , this);
		servers.record_calibration          = advertiseService("record_calibration"         , &FakeEnsensoNode::onRecordCalibration         , this);
		servers.finalize_calibration        = advertiseService("finalize_calibration"       , &FakeEnsensoNode::onFinalizeCalibration       , this);
		servers.set_workspace_calibration   = advertiseService("set_workspace_calibration"  , &FakeEnsensoNode::onSetWorkspaceCalibration   , this);
		servers.clear_workspace_calibration = advertiseService("clear_workspace_calibration", &FakeEnsensoNode::onClearWorkspaceCalibration , this);
		servers.calibrate_workspace         = advertiseService("calibrate"                  , &FakeEnsensoNode::onCalibrateWorkspace        , this);
		servers.store_workspace_calibration = advertiseService("store_calibration"          , &FakeEnsensoNode::onStoreWorkspaceCalibration , this);
		DR_SUCCESS("Fake Ensenso node initialized.");
	}

private:
	bool onGetData(dr_ensenso_msgs::GetCameraData::Request &, dr_ensenso_msgs::GetCameraData::Response & res) {
		DR_INFO("Received data request.");

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

		// publish point cloud if requested
		if (publish_cloud) {
			DR_SUCCESS("Publishing cloud");
			publishers.cloud.publish(res.point_cloud);
		}

		// publish image if requested
		if (publish_image) {
			DR_SUCCESS("Publishing image");
			std_msgs::Header header;
			header.frame_id = camera_frame;
			header.stamp = ros::Time::now();
			publishers.image.publish(res.color);
		}

		return true;
	}

	bool onDumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		DR_ERROR("The dump_data service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onGetPatternPose(dr_ensenso_msgs::GetPatternPose::Request &, dr_ensenso_msgs::GetPatternPose::Response &) {
		DR_ERROR("The get_pattern_pose service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onInitializeCalibration(dr_ensenso_msgs::InitializeCalibration::Request &, dr_ensenso_msgs::InitializeCalibration::Response &) {
		DR_ERROR("The initialize_calibration service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onRecordCalibration(dr_msgs::SendPose::Request &, dr_msgs::SendPose::Response &) {
		DR_ERROR("The record_calibration service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onFinalizeCalibration(dr_ensenso_msgs::FinalizeCalibration::Request &, dr_ensenso_msgs::FinalizeCalibration::Response &) {
		DR_ERROR("The finalize_calibration service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onSetWorkspaceCalibration(dr_msgs::SendPoseStamped::Request &, dr_msgs::SendPoseStamped::Response &) {
		DR_ERROR("The set_workspace service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onClearWorkspaceCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		DR_ERROR("The clear_workspace service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onCalibrateWorkspace(dr_ensenso_msgs::Calibrate::Request &, dr_ensenso_msgs::Calibrate::Response &) {
		DR_ERROR("The calibrate service is not implemented in the fake ensenso node.");
		return false;
	}

	bool onStoreWorkspaceCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		DR_ERROR("The store_calibration service is not implemented in the fake ensenso node.");
		return false;
	}

};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ensenso");
	dr::FakeEnsensoNode node;
	ros::spin();
}
