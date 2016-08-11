#include <dr_msgs/SendPose.h>
#include <dr_msgs/SendPoseStamped.h>
#include <dr_ensenso_msgs/EnsensoFinalizeCalibration.h>
#include <dr_ensenso_msgs/GetPatternPose.h>
#include <dr_ensenso_msgs/GetCameraData.h>
#include <dr_ensenso_msgs/EnsensoInitializeCalibration.h>

#include <dr_ensenso/ensenso.hpp>
#include <dr_ros/node.hpp>
#include <dr_util/timestamp.hpp>
#include <dr_eigen/ros.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace dr {

// this is needed because std::make_unique only exists from c++14
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class EnsensoNode: public Node {
public:
	EnsensoNode() : image_transport(*this) {
		configure();
		activate();
	}

private:
	/// Resets calibration state from this node.
	void resetCalibration() {
		moving_frame  = "";
		static_frame  = "";
		camera_guess  = boost::none;
		pattern_guess = boost::none;
		robot_poses.clear();
	}

protected:
	using Point = pcl::PointXYZ;
	using PointCloud = pcl::PointCloud<Point>;

	void configure() {
		// load ROS parameters
		param<std::string>("camera_frame", camera_frame, "camera_frame");
		param<std::string>("camera_data_path", camera_data_path, "camera_data");
		param<bool>("publish_images", publish_images, true);
		param<bool>("dump_images", dump_images, true);

		// get Ensenso serial
		serial = getParam<std::string>("serial", "");
		if (serial != "") {
			DR_INFO("Opening Ensenso with serial '" << serial << "'...");
		}

		try {
			// create the camera
			ensenso_camera = dr::make_unique<dr::Ensenso>(serial);
		} catch (dr::NxError const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		} catch (std::runtime_error const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		}
	}

	/// Activate ROS service servers and publishers
	void activate() {
		// activate service servers
		servers.camera_data            = advertiseService("get_data"              , &EnsensoNode::getData              , this);
		servers.dump_data              = advertiseService("dump_data"             , &EnsensoNode::dumpData             , this);
		servers.get_pattern_pose       = advertiseService("get_pattern_pose"      , &EnsensoNode::getPatternPose       , this);
		servers.initialize_calibration = advertiseService("initialize_calibration", &EnsensoNode::initializeCalibration, this);
		servers.record_calibration     = advertiseService("record_calibration"    , &EnsensoNode::recordCalibration    , this);
		servers.finalize_calibration   = advertiseService("finalize_calibration"  , &EnsensoNode::finalizeCalibration  , this);
		servers.set_workspace          = advertiseService("set_workspace"         , &EnsensoNode::setWorkspace         , this);
		servers.clear_workspace        = advertiseService("clear_workspace"       , &EnsensoNode::clearWorkspace       , this);

		// activate publishers
		publishers.calibration = advertise<geometry_msgs::PoseStamped>("calibration", 1, true);
		publishers.cloud       = advertise<PointCloud>("cloud", 1, true);
		publishers.image       = image_transport.advertise("image", 1, true);

		// load ensenso parameters file
		std::string ensenso_param_file = getParam<std::string>("ensenso_param_file", "");
		if (ensenso_param_file != "") {
			try {
				if (!ensenso_camera->loadParameters(ensenso_param_file)) {
					DR_ERROR("Failed to set Ensenso params. File path: " << ensenso_param_file);
				}
			} catch (dr::NxError const & e) {
				DR_ERROR("Failed to set Ensenso params. " << e.what());
			}
		}

		// load overlay parameters file
		std::string overlay_param_file = getParam<std::string>("overlay_param_file", "");
		if (overlay_param_file != "") {
			try {
				if (!ensenso_camera->loadOverlayParameters(overlay_param_file)) {
					DR_ERROR("Failed to set overlay camera params. File path: " << overlay_param_file);
				}
			} catch (dr::NxError const & e) {
				DR_ERROR("Failed to set overlay camera params. " << e.what());
			}
		}

		// load overlay parameter set file
		std::string overlay_param_set_file = getParam<std::string>("overlay_param_set_file", "");
		if (overlay_param_set_file != "") {
			try {
				ensenso_camera->loadOverlayParameterSet(overlay_param_set_file);
			} catch (dr::NxError const & e) {
				DR_ERROR("Failed to set overlay param set file. " << e.what());
			}
		}

		// initialize other member variables
		resetCalibration();

		// start publish calibration timer
		double calibration_timer_rate = getParam("calibration_timer_rate", -1);
		if (calibration_timer_rate > 0) {
			publish_calibration_timer = createTimer(ros::Duration(calibration_timer_rate), &EnsensoNode::publishCalibration, this);
		}

		// start image publishing timer
		double publish_images_rate = getParam("publish_images_rate", 30);
		if (publish_images_rate > 0) {
			publish_images_timer = createTimer(ros::Rate(publish_images_rate), &EnsensoNode::publishImage, this);
		}
	}

	void publishImage(ros::TimerEvent const &) {
		// create a header
		std_msgs::Header header;
		header.frame_id = camera_frame;
		header.stamp    = ros::Time::now();

		// prepare message
		cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, cv::Mat());

		// read the image
		try {
			ensenso_camera->loadIntensity(cv_image.image);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve image. " << e.what());
			return;
		}

		// publish the image
		publishers.image.publish(cv_image.toImageMsg());
	}

	PointCloud::Ptr getPointCloud() {
		PointCloud::Ptr cloud(new PointCloud);
		try {
			ensenso_camera->loadRegisteredPointCloud(*cloud);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve PointCloud. " << e.what());
			return nullptr;
		}
		cloud->header.frame_id = camera_frame;
		return cloud;
	}

	void dumpData(cv::Mat const & image, PointCloud::Ptr point_cloud) {
		// create path if it does not exist
		boost::filesystem::path path(camera_data_path);
		if (!boost::filesystem::is_directory(path)) {
			boost::filesystem::create_directory(camera_data_path);
		}

		std::string time_string = getTimeString();

		pcl::io::savePCDFile(camera_data_path + "/" + time_string + "_cloud.pcd", *point_cloud);
		cv::imwrite(camera_data_path + "/" + time_string + "_image.png", image);
	}

	bool getData(dr_ensenso_msgs::GetCameraData::Request &, dr_ensenso_msgs::GetCameraData::Response & res) {
		// prepare message
		cv_bridge::CvImage cv_image(res.point_cloud.header, sensor_msgs::image_encodings::BGR8, cv::Mat());

		// read the image
		try {
			ensenso_camera->loadIntensity(cv_image.image);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve image. " << e.what());
			return false;
		}
		res.color = *cv_image.toImageMsg();

		// read the point cloud
		PointCloud::Ptr point_cloud = getPointCloud();

		// convert to ROS message
		pcl::toROSMsg(*point_cloud, res.point_cloud);

		// store image and point cloud
		if (dump_images) {
			dumpData(cv_image.image, point_cloud);
		}

		// publish point cloud if requested
		if (publish_images) {
			publishers.cloud.publish(point_cloud);
		}

		return true;
	}

	bool dumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		// get image
		cv::Mat image;
		try {
			ensenso_camera->loadIntensity(image);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve image. " << e.what());
			return false;
		}
		cv::imwrite(getTimeString() + "_image.png", image);

		// dump the image and point cloud
		dumpData(image, getPointCloud());

		return true;
	}

	bool getPatternPose(dr_ensenso_msgs::GetPatternPose::Request & req, dr_ensenso_msgs::GetPatternPose::Response & res) {
		try {
			Eigen::Isometry3d pattern_pose;

			if (!ensenso_camera->getPatternPose(pattern_pose, req.samples)) {
				DR_ERROR("Failed to get pattern pose.");
				return false;
			}

			res.data = dr::toRosPose(pattern_pose);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to find calibration pattern. " << e.what());
			return false;
		}

		return true;
	}

	bool initializeCalibration(dr_ensenso_msgs::EnsensoInitializeCalibration::Request & req, dr_ensenso_msgs::EnsensoInitializeCalibration::Response &) {
		try {
			ensenso_camera->discardPatterns();
			ensenso_camera->clearWorkspace();
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to discard patterns. " << e.what());
			return false;
		}
		resetCalibration();

		camera_moving = req.camera_moving;
		moving_frame  = req.moving_frame;
		static_frame  = req.static_frame;

		// check for valid camera guess
		if (req.camera_guess.position.x == 0 && req.camera_guess.position.y == 0 && req.camera_guess.position.z == 0 &&
			req.camera_guess.orientation.x == 0 && req.camera_guess.orientation.y == 0 && req.camera_guess.orientation.z == 0 && req.camera_guess.orientation.w == 0) {
			camera_guess = boost::none;
		} else {
			camera_guess = dr::toEigen(req.camera_guess);
		}

		// check for valid pattern guess
		if (req.pattern_guess.position.x == 0 && req.pattern_guess.position.y == 0 && req.pattern_guess.position.z == 0 &&
			req.pattern_guess.orientation.x == 0 && req.pattern_guess.orientation.y == 0 && req.pattern_guess.orientation.z == 0 && req.pattern_guess.orientation.w == 0) {
			pattern_guess = boost::none;
		} else {
			pattern_guess = dr::toEigen(req.pattern_guess);
		}

		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			DR_ERROR("No calibration frame provided.");
			return false;
		}

		DR_INFO("Successfully initialized calibration sequence.");

		return true;
	}

	bool recordCalibration(dr_msgs::SendPose::Request & req, dr_msgs::SendPose::Response &) {
		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			DR_ERROR("No calibration frame provided.");
			return false;
		}

		try {
			// record a pattern
			ensenso_camera->recordCalibrationPattern();

			// add robot pose to list of poses
			robot_poses.push_back(dr::toEigen(req.data));
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to record calibration pattern. " << e.what());
			return false;
		}

		DR_INFO("Successfully recorded a calibration sample.");
		return true;
	}

	bool finalizeCalibration(dr_ensenso_msgs::EnsensoFinalizeCalibration::Request &, dr_ensenso_msgs::EnsensoFinalizeCalibration::Response & res) {
		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			DR_ERROR("No calibration frame provided.");
			return false;
		}

		try {
			// perform calibration
			dr::Ensenso::CalibrationResult calibration =
				ensenso_camera->computeCalibration(robot_poses, camera_moving, camera_guess, pattern_guess, camera_moving ? moving_frame : static_frame);

			// copy result
			res.camera_pose        = dr::toRosPoseStamped(std::get<0>(calibration), camera_moving ? moving_frame : static_frame);
			res.pattern_pose       = dr::toRosPoseStamped(std::get<1>(calibration), camera_moving ? static_frame : moving_frame);
			res.iterations         = std::get<2>(calibration);
			res.reprojection_error = std::get<3>(calibration);

			// store result in camera
			ensenso_camera->storeCalibration();

			// clear state
			resetCalibration();
		} catch (dr::NxError const & e) {
			// clear state (?)
			resetCalibration();

			DR_ERROR("Failed to finalize calibration. " << e.what());
			return false;
		}

		DR_INFO("Successfully finished calibration sequence.");

		publishCalibration();
		return true;
	}

	bool setWorkspace(dr_msgs::SendPoseStamped::Request & req, dr_msgs::SendPoseStamped::Response &) {
		try {
			ensenso_camera->setWorkspace(dr::toEigen(req.data.pose), req.data.header.frame_id);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to set workspace. " << e.what());
			return false;
		}
		return true;
	}

	bool clearWorkspace(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->clearWorkspace();
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to clear camera pose. " << e.what());
			return false;
		}
		return true;
	}

	void publishCalibration(ros::TimerEvent const & = {}) {
		geometry_msgs::PoseStamped pose;
		boost::optional<std::string> frame = ensenso_camera->getFrame();
		if (frame) {
			pose = dr::toRosPoseStamped(ensenso_camera->getCameraPose()->inverse(), *frame, ros::Time::now());
		} else {
			pose = dr::toRosPoseStamped(Eigen::Isometry3d::Identity(), "", ros::Time::now());
		}

		publishers.calibration.publish(pose);
	}

	/// The wrapper for the Ensenso stereo camera.
	std::unique_ptr<dr::Ensenso> ensenso_camera;

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
		ros::ServiceServer set_workspace;

		/// Service server for clearing the camera pose setting of the Ensenso.
		ros::ServiceServer clear_workspace;
	} servers;

	/// Object for handling transportation of images.
	image_transport::ImageTransport image_transport;

	/// Timer to trigger calibration publishing.
	ros::Timer publish_calibration_timer;

	/// Timer to trigger image publishing.
	ros::Timer publish_images_timer;

	struct Publishers {
		/// Publisher for the calibration result.
		ros::Publisher calibration;

		/// Publisher for publishing raw point clouds.
		ros::Publisher cloud;

		/// Publisher for publishing images.
		image_transport::Publisher image;
	} publishers;

	/// The calibrated pose of the camera.
	geometry_msgs::PoseStamped camera_pose;

	/// The frame in which the image and point clouds are send.
	std::string camera_frame;

	/// Frame to calibrate the camera to when camera_moving is true (gripper frame).
	std::string moving_frame;

	/// Frame to calibrate the camera to when camera_moving is false (robot origin or world frame).
	std::string static_frame;

	/// Serial id of the Ensenso camera.
	std::string serial;

	/// If true, publishes images with a frequency of 30Hz.
	bool publish_images;

	/// If true, dump recorded images.
	bool dump_images;

	// Guess of the camera pose relative to gripper (for moving camera) or relative to robot origin (for static camera).
	boost::optional<Eigen::Isometry3d> camera_guess;

	// Guess of the calibration pattern pose relative to gripper (for static camera) or relative to robot origin (for moving camera).
	boost::optional<Eigen::Isometry3d> pattern_guess;

	/// Used in calibration. Determines if the camera is moving (eye in hand) or static.
	bool camera_moving;

	/// List of robot poses corresponding to the list of recorded calibration patterns.
	std::vector<Eigen::Isometry3d> robot_poses;

	/// Location where the images and point clouds are stored.
	std::string camera_data_path;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ensenso_node");
	dr::EnsensoNode node;
	ros::spin();
}

