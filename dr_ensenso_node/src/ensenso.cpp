#include "timestamp.hpp"

#include <dr_eigen/ros.hpp>
#include <dr_eigen/yaml.hpp>
#include <dr_ensenso/ensenso.hpp>
#include <dr_ensenso/opencv.hpp>
#include <dr_ensenso/util.hpp>
#include <dr_param/param.hpp>

#include <dr_ensenso_msgs/Calibrate.h>
#include <dr_ensenso_msgs/FinalizeCalibration.h>
#include <dr_ensenso_msgs/GetCameraData.h>
#include <dr_ensenso_msgs/GetCameraParams.h>
#include <dr_ensenso_msgs/SendPoseStamped.h>
#include <dr_ensenso_msgs/DetectCalibrationPattern.h>
#include <dr_ensenso_msgs/InitializeCalibration.h>
#include <dr_ensenso_msgs/SendPose.h>
#include <dr_ensenso_msgs/SendPoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp>

#include <memory>

namespace {

NxLibItem selectNxItmCalibration(std::string const & camera, dr::Ensenso const & ensenso_camera) {
	NxLibItem item;
	if (camera == dr_ensenso_msgs::GetCameraParams::Request::MONO) {
		if (!ensenso_camera.hasMonocular()) throw std::runtime_error("No monocular camera available!");
		return (*(ensenso_camera.nativeMonocular()))[itmCalibration];
	} else if (camera == "Left") {
		return ensenso_camera.native()[itmCalibration][itmMonocular][itmLeft];
	} else if (camera == "Right") {
		return ensenso_camera.native()[itmCalibration][itmMonocular][itmRight];
	} else {
		throw std::runtime_error("Invalid camera requested.");
	}
}

cv::Mat getCameraParams(dr_ensenso_msgs::GetCameraParams::Request const & req, dr::Ensenso const & ensenso_camera) {
	switch (req.type) {
		case dr_ensenso_msgs::GetCameraParams::Request::CAMERA_MATRIX: {
			return dr::toCameraMatrix(selectNxItmCalibration(req.camera, ensenso_camera));
		} case dr_ensenso_msgs::GetCameraParams::Request::DISTORTION_MATRIX: {
			return dr::toDistortionParameters(selectNxItmCalibration(req.camera, ensenso_camera));
		} case dr_ensenso_msgs::GetCameraParams::Request::RECTIFICATION_MATRIX: {
			if (req.camera == dr_ensenso_msgs::GetCameraParams::Request::MONO) {
				throw std::runtime_error("Request for mono camera rectification matrix is unsupported!");
			}
			return dr::toRectificationMatrix(ensenso_camera.native()[itmCalibration][itmStereo][req.camera == "Left" ? itmLeft : itmRight]);
		} case dr_ensenso_msgs::GetCameraParams::Request::PROJECTION_MATRIX: {
			if (req.camera == dr_ensenso_msgs::GetCameraParams::Request::MONO) {
				throw std::runtime_error("Request for mono camera projection matrix is unsupported!");
			}
			return dr::toProjectionMatrix(ensenso_camera.native()[itmCalibration], req.camera);
		} default: {
			throw std::runtime_error("Invalid type requested!");
		}
	}
}

}

namespace dr {

// this is needed because std::make_unique only exists from c++14
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class EnsensoNode: public ros::NodeHandle {
public:
	EnsensoNode() : ros::NodeHandle("~"), image_transport(*this) {
		configure();
	}

private:
	/// Resets calibration state from this node.
	void resetCalibration() {
		moving_frame  = "";
		fixed_frame   = "";
		camera_guess  = boost::none;
		pattern_guess = boost::none;
		robot_poses.clear();
	}

	ros::NodeHandle       & handle()       { return *this; }
	ros::NodeHandle const & handle() const { return *this; }

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
		param<bool>("connect_monocular", connect_monocular, true);
		param<bool>("use_frontlight", use_frontlight, true);
		param<bool>("synced_retrieve", synced_retrieve, false);

		// get Ensenso serial
		serial = dr::getParam<std::string>(handle(), "serial", "");
		if (serial != "") {
			ROS_INFO_STREAM("Opening Ensenso with serial '" << serial << "'...");
		} else {
			ROS_INFO_STREAM("Opening first available Ensenso...");
		}

		try {
			// create the camera
			ensenso_camera = dr::make_unique<dr::Ensenso>(serial, connect_monocular);
		} catch (dr::NxError const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		} catch (std::runtime_error const & e) {
			throw std::runtime_error("Failed initializing camera. " + std::string(e.what()));
		}

		// activate service servers
		servers.camera_data                 = advertiseService("get_data"                    , &EnsensoNode::onGetData                  , this);
		servers.dump_data                   = advertiseService("dump_data"                   , &EnsensoNode::onDumpData                 , this);
		servers.get_pattern_pose            = advertiseService("detect_calibration_pattern"  , &EnsensoNode::onDetectCalibrationPattern , this);
		servers.initialize_calibration      = advertiseService("initialize_calibration"      , &EnsensoNode::onInitializeCalibration    , this);
		servers.record_calibration          = advertiseService("record_calibration"          , &EnsensoNode::onRecordCalibration        , this);
		servers.finalize_calibration        = advertiseService("finalize_calibration"        , &EnsensoNode::onFinalizeCalibration      , this);
		servers.set_workspace_calibration   = advertiseService("set_workspace_calibration"   , &EnsensoNode::onSetWorkspaceCalibration  , this);
		servers.clear_workspace_calibration = advertiseService("clear_workspace_calibration" , &EnsensoNode::onClearWorkspaceCalibration, this);
		servers.calibrate_workspace         = advertiseService("calibrate_workspace"         , &EnsensoNode::onCalibrateWorkspace       , this);
		servers.store_workspace_calibration = advertiseService("store_workspace_calibration" , &EnsensoNode::onStoreWorkspaceCalibration, this);
		servers.get_camera_params           = advertiseService("get_camera_params"           , &EnsensoNode::onGetCameraParams          , this);

		// activate publishers
		publishers.calibration = advertise<geometry_msgs::PoseStamped>("calibration", 1, true);
		publishers.cloud       = advertise<PointCloud>("cloud", 1, true);
		publishers.image       = image_transport.advertise("image", 1, true);

		// load ensenso parameters file
		std::string ensenso_param_file = dr::getParam<std::string>(handle(), "ensenso_param_file", "");
		if (ensenso_param_file != "") {
			try {
				if (!ensenso_camera->loadParameters(ensenso_param_file)) {
					ROS_ERROR_STREAM("Failed to set Ensenso params. File path: " << ensenso_param_file);
				}
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set Ensenso params. " << e.what());
			}
		}

		// load monocular parameters file
		std::string monocular_param_file = dr::getParam<std::string>(handle(), "monocular_param_file", "");
		if (monocular_param_file != "") {
			try {
				if (!ensenso_camera->loadMonocularParameters(monocular_param_file)) {
					ROS_ERROR_STREAM("Failed to set monocular camera params. File path: " << monocular_param_file);
				}
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set monocular camera params. " << e.what());
			}
		}

		// load monocular parameter set file
		std::string monocular_ueye_param_file = dr::getParam<std::string>(handle(), "monocular_ueye_param_file", "");
		if (monocular_ueye_param_file != "") {
			try {
				ensenso_camera->loadMonocularUeyeParameters(monocular_ueye_param_file);
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set monocular param set file. " << e.what());
			}
		}

		// initialize other member variables
		resetCalibration();

		// start publish calibration timer
		double calibration_timer_rate = dr::getParam(handle(), "calibration_timer_rate", -1);
		if (calibration_timer_rate > 0) {
			publish_calibration_timer = createTimer(ros::Duration(calibration_timer_rate), &EnsensoNode::publishCalibration, this);
		}

		// start image publishing timer
		double publish_images_rate = dr::getParam(handle(), "publish_images_rate", 30);
		if (publish_images_rate > 0) {
			publish_images_timer = createTimer(ros::Rate(publish_images_rate), &EnsensoNode::publishImage, this);
		}

		// check if there is an monocular camera connected
		has_monocular = ensenso_camera->hasMonocular();

		// check if camera really has front light. This will throw an error if it doesn't.
		if (use_frontlight) ensenso_camera->setFrontLight(false);

		ROS_INFO_STREAM("Ensenso opened successfully.");
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
			has_monocular ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::MONO8,
			getImage(!has_monocular)
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
			ROS_ERROR_STREAM("Failed to retrieve PointCloud. " << e.what());
			return nullptr;
		}
		cloud->header.frame_id = camera_frame;

		return cloud;
	}

	cv::Mat getImage(bool capture) {
		// get a grayscale image? then enable frontlight
		int flex_view;
		if (!has_monocular && capture) {
			flex_view = ensenso_camera->flexView();
			ensenso_camera->setFlexView(0);
			ensenso_camera->setProjector(false);
			if (use_frontlight) ensenso_camera->setFrontLight(true);
		}

		cv::Mat image;
		try {
			ensenso_camera->loadIntensity(image, capture);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve image. " << e.what());
			return cv::Mat();
		}

		// restore settings
		if (!has_monocular && capture) {
			if (use_frontlight) ensenso_camera->setFrontLight(false);
			ensenso_camera->setProjector(true);
			if (flex_view > 0) {
				ensenso_camera->setFlexView(flex_view);
			}
		}

		return image;
	}

	void dumpData(PointCloud::ConstPtr point_cloud, cv::Mat const & image) {
		// create path if it does not exist
		boost::filesystem::path path(camera_data_path);
		if (!boost::filesystem::is_directory(path)) {
			boost::filesystem::create_directory(camera_data_path);
		}

		std::string time_string = getTimeString();

		pcl::io::savePCDFileBinary(camera_data_path + "/" + time_string + "_cloud.pcd", *point_cloud);
		cv::imwrite(camera_data_path + "/" + time_string + "_image.png", image);
	}

	bool capture(bool stereo, bool monocular) {
		// retrieve image data
		try {
			if (!ensenso_camera->retrieve(true, 3000, stereo, has_monocular && monocular)) {
				ROS_ERROR_STREAM("Failed to retrieve image data.");
				return false;
			}
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve image data. " << e.what());
			return false;
		}

		return true;
	}

	boost::optional<Data> getData() {
		cv::Mat image;

		// when using an monocular, capture both simultaneously
		if (has_monocular) {
			if (!capture(synced_retrieve, true)) return boost::none;

			// capture stereo image after monocular image if we don't synchronize the triggers
			if (!synced_retrieve && !capture(true, false)) return boost::none;
			image = getImage(false);

		// when not using an monocular, capture image first
		} else {
			image = getImage(true);
			if (!capture(true, false)) return boost::none;
		}

		return Data{getPointCloud(), image};
	}

	bool onGetData(dr_ensenso_msgs::GetCameraData::Request &, dr_ensenso_msgs::GetCameraData::Response & res) {
		boost::optional<Data> data = getData();
		if (!data) return false;
		pcl::toROSMsg(*data->cloud, res.point_cloud);

		// get the image
		cv_bridge::CvImage cv_image(
			res.point_cloud.header,
			has_monocular ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::MONO8,
			data->image
		);
		res.color = *cv_image.toImageMsg();

		// store image and point cloud
		if (dump_images) dumpData(data->cloud, data->image);

		// publish point cloud if requested
		if (publish_cloud) {
			publishers.cloud.publish(data->cloud);
		}

		return true;
	}

	bool onDumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		boost::optional<Data> data = getData();
		if (!data) return false;
		dumpData(data->cloud, data->image);
		return true;
	}

	bool onDetectCalibrationPattern(dr_ensenso_msgs::DetectCalibrationPattern::Request & req, dr_ensenso_msgs::DetectCalibrationPattern::Response & res) {
		if (req.samples == 0) {
			ROS_ERROR_STREAM("Unable to get pattern pose. Number of samples is set to 0.");
			return false;
		}

		try {
			res.data = dr::toRosPose(ensenso_camera->detectCalibrationPattern(req.samples, req.ignore_calibration));
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to find calibration pattern. " << e.what());
			return false;
		}

		return true;
	}

	bool onInitializeCalibration(dr_ensenso_msgs::InitializeCalibration::Request & req, dr_ensenso_msgs::InitializeCalibration::Response &) {
		try {
			ensenso_camera->discardCalibrationPatterns();
			ensenso_camera->clearWorkspaceCalibration();
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to discard patterns. " << e.what());
			return false;
		}
		resetCalibration();

		camera_moving = req.camera_moving;
		moving_frame  = req.moving_frame;
		fixed_frame   = req.fixed_frame;

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
		if (moving_frame == "" || fixed_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		ROS_INFO_STREAM("Successfully initialized calibration sequence.");

		return true;
	}

	bool onRecordCalibration(dr_ensenso_msgs::SendPose::Request & req, dr_ensenso_msgs::SendPose::Response &) {
		// check for proper initialization
		if (moving_frame == "" || fixed_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		try {
			// record a pattern
			ensenso_camera->recordCalibrationPattern();

			// add robot pose to list of poses
			robot_poses.push_back(dr::toEigen(req.data));
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to record calibration pattern. " << e.what());
			return false;
		}

		ROS_INFO_STREAM("Successfully recorded a calibration sample.");
		return true;
	}

	bool onFinalizeCalibration(dr_ensenso_msgs::FinalizeCalibration::Request &, dr_ensenso_msgs::FinalizeCalibration::Response & res) {
		// check for proper initialization
		if (moving_frame == "" || fixed_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		try {
			// perform calibration
			dr::Ensenso::CalibrationResult calibration =
				ensenso_camera->computeCalibration(robot_poses, camera_moving, camera_guess, pattern_guess, camera_moving ? moving_frame : fixed_frame);

			// copy result
			res.camera_pose        = dr::toRosPoseStamped(std::get<0>(calibration), camera_moving ? moving_frame : fixed_frame);
			res.pattern_pose       = dr::toRosPoseStamped(std::get<1>(calibration), camera_moving ? fixed_frame : moving_frame);
			res.iterations         = std::get<2>(calibration);
			res.reprojection_error = std::get<3>(calibration);

			// store result in camera
			ensenso_camera->storeWorkspaceCalibration();

			// clear state
			resetCalibration();
		} catch (dr::NxError const & e) {
			// clear state (?)
			resetCalibration();

			ROS_ERROR_STREAM("Failed to finalize calibration. " << e.what());
			return false;
		}

		ROS_INFO_STREAM("Successfully finished calibration sequence.");
		return true;
	}

	bool onSetWorkspaceCalibration(dr_ensenso_msgs::SendPoseStamped::Request & req, dr_ensenso_msgs::SendPoseStamped::Response &) {
		try {
			ensenso_camera->setWorkspaceCalibration(dr::toEigen(req.data.pose), req.data.header.frame_id, Eigen::Isometry3d::Identity());
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to set workspace calibration: " << e.what());
			return false;
		}
		return true;
	}

	bool onClearWorkspaceCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->clearWorkspaceCalibration();
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to clear workspace calibration: " << e.what());
			return false;
		}
		return true;
	}

	bool onCalibrateWorkspace(dr_ensenso_msgs::Calibrate::Request & req, dr_ensenso_msgs::Calibrate::Response &) {
		ROS_INFO_STREAM("Performing workspace calibration.");

		if (req.frame_id.empty()) {
			ROS_ERROR_STREAM("Calibration frame not set. Can not calibrate.");
			return false;
		}

		try {
			Eigen::Isometry3d pattern_pose = ensenso_camera->detectCalibrationPattern(req.samples);
			ROS_INFO_STREAM("Found calibration pattern at:\n" << dr::toYaml(pattern_pose));
			ROS_INFO_STREAM("Defined pattern pose:\n" << dr::toYaml(dr::toEigen(req.pattern)));
			ensenso_camera->setWorkspaceCalibration(pattern_pose, req.frame_id, dr::toEigen(req.pattern), true);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to calibrate camera pose. " << e.what());
			return false;
		}
		return true;
	}

	bool onStoreWorkspaceCalibration(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->storeWorkspaceCalibration();
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to store calibration. " << e.what());
			return false;
		}
		return true;
	}

	bool onGetCameraParams(dr_ensenso_msgs::GetCameraParams::Request & req, dr_ensenso_msgs::GetCameraParams::Response & res) {
		if (true
			&& req.camera != dr_ensenso_msgs::GetCameraParams::Request::MONO
			&& req.camera != dr_ensenso_msgs::GetCameraParams::Request::LEFT
			&& req.camera != dr_ensenso_msgs::GetCameraParams::Request::RIGHT
		) {
			ROS_ERROR_STREAM("Invalid camera type " << req.camera << " requested.");
			return false;
		}

		if (req.camera == dr_ensenso_msgs::GetCameraParams::Request::MONO && !ensenso_camera->hasMonocular()) {
			ROS_ERROR_STREAM("No monocular camera available!");
			return false;
		}

		try {
			cv::Mat mat = getCameraParams(req, *ensenso_camera);
			for (int i = 0; i < mat.rows; i++) {
				for (int j = 0; j < mat.cols; j++) {
					res.params.push_back(mat.at<double>(i, j));
				}
			}
		} catch (std::runtime_error const & e) {
			ROS_ERROR_STREAM(e.what());
			return false;
		}

		return true;
	}

	void publishCalibration(ros::TimerEvent const &) {
		geometry_msgs::PoseStamped pose;
		std::string frame = ensenso_camera->getWorkspaceCalibrationFrame();
		if (!frame.empty()) {
			pose = dr::toRosPoseStamped(ensenso_camera->getWorkspaceCalibration()->inverse(), frame, ros::Time::now());
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
		ros::ServiceServer set_workspace_calibration;

		/// Service server for clearing the camera pose setting of the Ensenso.
		ros::ServiceServer clear_workspace_calibration;

		/// Service server combining 'get_pattern_pose', 'set_workspace', and stores it to the ensenso.
		ros::ServiceServer calibrate_workspace;

		/// Service server for storing the calibration.
		ros::ServiceServer store_workspace_calibration;

		/// Service server for retrieving a camera's parameters.
		ros::ServiceServer get_camera_params;
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
	std::string fixed_frame;

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

	/// If true, the Ensenso has an monocular camera connected.
	bool has_monocular;

	/// If true, tries to connect an monocular camera.
	bool connect_monocular;

	/// If true, the front light is available.
	bool use_frontlight;

	/// If true, retrieves the monocular camera and Ensenso simultaneously. A hardware trigger is advised to remove the projector from the uEye image.
	bool synced_retrieve;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ensenso");
	dr::EnsensoNode node;
	ros::spin();
}

