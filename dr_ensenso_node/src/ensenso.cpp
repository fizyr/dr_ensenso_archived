#include <dr_msgs/SendPose.h>
#include <dr_msgs/SendPoseStamped.h>
#include <dr_ensenso_msgs/EnsensoFinalizeCalibration.h>
#include <dr_ensenso_msgs/GetPatternPose.h>
#include <dr_ensenso_msgs/GetCameraData.h>
#include <dr_ensenso_msgs/EnsensoInitializeCalibration.h>
#include <dr_ensenso_msgs/Calibrate.h>

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

#include <boost/optional.hpp>

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

	struct Data {
		PointCloud::Ptr cloud;
		cv::Mat image;
	};

	void configure() {
		// load ROS parameters
		param<std::string>("camera_frame", camera_frame, "camera_frame");
		param<std::string>("camera_data_path", camera_data_path, "camera_data");
		param<bool>("publish_cloud", publish_cloud, true);
		param<bool>("dump_images", dump_images, true);
		param<bool>("registered", registered, true);
		param<bool>("connect_overlay", connect_overlay, true);

		// get Ensenso serial
		serial = getParam<std::string>("serial", "");
		if (serial != "") {
			DR_INFO("Opening Ensenso with serial '" << serial << "'...");
		} else {
			DR_INFO("Opening first available Ensenso...");
		}

		try {
			// create the camera
			ensenso_camera = dr::make_unique<dr::Ensenso>(serial, connect_overlay);
		} catch (dr::NxError const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		} catch (std::runtime_error const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		}

		// activate service servers
		servers.camera_data            = advertiseService("get_data"              , &EnsensoNode::getData              , this);
		servers.dump_data              = advertiseService("dump_data"             , &EnsensoNode::dumpData             , this);
		servers.get_pattern_pose       = advertiseService("get_pattern_pose"      , &EnsensoNode::getPatternPose       , this);
		servers.initialize_calibration = advertiseService("initialize_calibration", &EnsensoNode::initializeCalibration, this);
		servers.record_calibration     = advertiseService("record_calibration"    , &EnsensoNode::recordCalibration    , this);
		servers.finalize_calibration   = advertiseService("finalize_calibration"  , &EnsensoNode::finalizeCalibration  , this);
		servers.set_workspace          = advertiseService("set_workspace_calibration", &EnsensoNode::setWorkspaceCalibration, this);
		servers.clear_workspace        = advertiseService("clear_workspace_calibration", &EnsensoNode::clearWorkspaceCalibration, this);
		servers.calibrate              = advertiseService("calibrate"             , &EnsensoNode::calibrate            , this);
		servers.store_calibration      = advertiseService("store_calibration"     , &EnsensoNode::storeCalibration     , this);

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

		// check if there is an overlay camera connected
		has_overlay = ensenso_camera->hasOverlay();

		DR_SUCCESS("Ensenso opened successfully.");
	}

	void publishImage(ros::TimerEvent const &) {
		if (publishers.image.getNumSubscribers() == 0) return;

		// capture only image
		capture(false, true);

		// create a header
		std_msgs::Header header;
		header.frame_id = camera_frame;
		header.stamp    = ros::Time::now();

		// prepare message
		cv_bridge::CvImage cv_image(
			header,
			has_overlay ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::MONO8,
			getImage(!has_overlay)
		);

		// publish the image
		publishers.image.publish(cv_image.toImageMsg());
	}

	PointCloud::Ptr getPointCloud() {
		PointCloud::Ptr cloud(new PointCloud);
		try {
			if (registered) {
				ensenso_camera->loadRegisteredPointCloud(*cloud, cv::Rect(), false);
			} else {
				ensenso_camera->loadPointCloud(*cloud, cv::Rect(), false);
			}
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve PointCloud. " << e.what());
			return nullptr;
		}
		cloud->header.frame_id = camera_frame;

		return cloud;
	}

	cv::Mat getImage(bool capture) {
		// get a grayscale image? then enable frontlight
		int flex_view;
		if (!has_overlay && capture) {
			flex_view = ensenso_camera->flexView();
			ensenso_camera->setFlexView(0);
			ensenso_camera->setProjector(false);
			ensenso_camera->setFrontLight(true);
		}

		cv::Mat image;
		try {
			ensenso_camera->loadIntensity(image, capture);
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve image. " << e.what());
			return cv::Mat();
		}

		// restore settings
		if (!has_overlay && capture) {
			ensenso_camera->setFrontLight(false);
			ensenso_camera->setProjector(true);
			if (flex_view > 0) {
				ensenso_camera->setFlexView(flex_view);
			}
		}

		return image;
	}

	void dumpData(PointCloud::Ptr point_cloud, cv::Mat const & image) {
		// create path if it does not exist
		boost::filesystem::path path(camera_data_path);
		if (!boost::filesystem::is_directory(path)) {
			boost::filesystem::create_directory(camera_data_path);
		}

		std::string time_string = getTimeString();

		pcl::io::savePCDFile(camera_data_path + "/" + time_string + "_cloud.pcd", *point_cloud);
		cv::imwrite(camera_data_path + "/" + time_string + "_image.png", image);
	}

	bool capture(bool stereo, bool overlay) {
		// retrieve image data
		try {
			if (!ensenso_camera->retrieve(true, 3000, stereo, has_overlay && overlay)) {
				DR_ERROR("Failed to retrieve image data.");
				return false;
			}
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to retrieve image data. " << e.what());
			return false;
		}

		return true;
	}

	boost::optional<Data> getData() {
		cv::Mat image;

		// when using an overlay, capture both simultaneously
		if (has_overlay) {
			if (!capture(true, true)) return boost::none;
			image = getImage(false);

		// when not using an overlay, capture image first
		} else {
			image = getImage(true);
			if (!capture(true, false)) return boost::none;
		}

		return Data{getPointCloud(), image};
	}

	bool getData(dr_ensenso_msgs::GetCameraData::Request &, dr_ensenso_msgs::GetCameraData::Response & res) {
		boost::optional<Data> data = getData();
		if (!data) return false;
		pcl::toROSMsg(*data->cloud, res.point_cloud);

		// get the image
		cv_bridge::CvImage cv_image(
			res.point_cloud.header,
			has_overlay ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::MONO8,
			data->image
		);
		res.color = *cv_image.toImageMsg();

		// store image and point cloud
		if (dump_images) {
			dumpData(data->cloud, data->image);
		}

		// publish point cloud if requested
		if (publish_cloud) {
			publishers.cloud.publish(data->cloud);
		}

		return true;
	}

	bool dumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		boost::optional<Data> data = getData();
		if (!data) return false;

		dumpData(data->cloud, data->image);
		return true;
	}

	bool getPatternPose(dr_ensenso_msgs::GetPatternPose::Request & req, dr_ensenso_msgs::GetPatternPose::Response & res) {

		if (req.samples == 0) {
			DR_ERROR("Unable to get pattern pose. Number of samples is set to 0.");
			return false;
		}

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
		return true;
	}

	bool setWorkspace(Eigen::Isometry3d const & pattern_pose, std::string const & frame_id, Eigen::Isometry3d const & defined_pose) {
		if (frame_id == "") {
			DR_ERROR("Workspace frame id is empty");
			return false;
		}
		ensenso_camera->clearWorkspace();
		ensenso_camera->setWorkspace(pattern_pose, frame_id, defined_pose);
		return true;
	}


	bool setWorkspaceCalibration(dr_msgs::SendPoseStamped::Request & req, dr_msgs::SendPoseStamped::Response &) {
		try {
			if (!setWorkspace(dr::toEigen(req.data.pose), req.data.header.frame_id, Eigen::Isometry3d::Identity())) {
				DR_ERROR("Failed to set work space");
				return false;
			}
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to set workspace. " << e.what());
			return false;
		}
		return true;
	}

	bool clearWorkspaceCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->clearWorkspace();
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to clear camera pose. " << e.what());
			return false;
		}
		return true;
	}

	bool calibrate(dr_ensenso_msgs::Calibrate::Request & req, dr_ensenso_msgs::Calibrate::Response &) {
		try {
			Eigen::Isometry3d pattern_pose;
			if (!ensenso_camera->getPatternPose(pattern_pose, req.samples)) {
				DR_ERROR("Failed to get pattern pose.");
				return false;
			}

			if (!setWorkspace(pattern_pose, req.frame_id, dr::toEigen(req.workspace))) {
				DR_ERROR("Failed to set workspace.");
				return false;
			}
			ensenso_camera->storeCalibration();
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to calibrate camera pose. " << e.what());
			return false;
		}
		return true;
	}

	bool storeCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->storeCalibration();
		} catch (dr::NxError const & e) {
			DR_ERROR("Failed to store calibration. " << e.what());
			return false;
		}
		return true;
	}

	void publishCalibration(ros::TimerEvent const &) {
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

		/// Service server combining 'get_pattern_pose', 'set_workspace', and stores it to the ensenso.
		ros::ServiceServer calibrate;

		/// Service server for storing the calibration.
		ros::ServiceServer store_calibration;
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

	/// If true, publishes point cloud data when calling getData.
	bool publish_cloud;

	/// If true, dump recorded images.
	bool dump_images;

	/// If true, registers the point clouds.
	bool registered;

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

	/// If true, the Ensenso has an overlay camera connected.
	bool has_overlay;

	/// If true, tries to connect an overlay camera.
	bool connect_overlay;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ensenso_node");
	dr::EnsensoNode node;
	ros::spin();
}

