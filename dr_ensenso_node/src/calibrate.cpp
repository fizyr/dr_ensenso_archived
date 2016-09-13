#include <dr_ensenso_msgs/Calibrate.h>

#include <dr_eigen/ros.hpp>
#include <dr_eigen/yaml.hpp>
#include <dr_ros/node.hpp>
#include <dr_ros/service_client.hpp>
#include <dr_ros/subscriber.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace dr {
namespace ensenso {

geometry_msgs::Point vector3ToPoint(geometry_msgs::Vector3 const & vector) {
	geometry_msgs::Point result;
	result.x = vector.x;
	result.y = vector.y;
	result.z = vector.z;
	return result;
}

geometry_msgs::PoseStamped transformToPose(geometry_msgs::TransformStamped const & transform) {
	geometry_msgs::PoseStamped result;
	result.header           = transform.header;
	result.pose.position    = vector3ToPoint(transform.transform.translation);
	result.pose.orientation = transform.transform.rotation;
	return result;
}

geometry_msgs::PoseStamped lookupPose(tf2::BufferCore const & tf, std::string const & parent, std::string const & child, ros::Time const & time) {
	return transformToPose(tf.lookupTransform(parent, child, time));
}

}}

int main(int argc, char * * argv) {
	ros::init(argc, argv, "ensenso_calibration_tool");

	if (argc < 3) {
		std::cerr << "Usage: " << argv[0] << " calibration_frame tag_frame [samples]\n"
			<< "\n"
			<< "Note: If samples is not given, it defaults to 10.\n";
		return 1;
	}

	std::string calibration_frame = argv[1];
	std::string tag_frame         = argv[2];
	int samples                   = argc >= 4 ? std::stoi(argv[3]) : 10;
	std::string service           = "/ensenso/calibrate_workspace";

	tf2_ros::Buffer tf;
	tf2_ros::TransformListener tf_listener{tf};

	for (int i =0; ros::ok(); i = (i + 1) % 10) {
		std::string tf_error;
		if (tf.canTransform(tag_frame, calibration_frame, ros::Time(0), ros::Duration(0.1), &tf_error)) break;
		if (i == 9) {
			std::cerr << "TF lookup failed, retrying: " << tf_error << "\n";
		}
	}

	if (!ros::ok()) return 1;

	geometry_msgs::PoseStamped pose = dr::ensenso::lookupPose(tf, calibration_frame, tag_frame, ros::Time(0));

	std::cerr << "Tag pose in calibration frame:\n" << dr::toYaml(dr::toEigen(pose.pose)) << "\n";

	dr_ensenso_msgs::Calibrate calibrate;
	calibrate.request.frame_id = calibration_frame;
	calibrate.request.samples  = samples;
	calibrate.request.pattern  = pose.pose;

	if (!ros::service::exists(service, false)) {
		std::cerr << "Service `" << service << "' not available yet. Waiting.\n";
		if (!ros::service::waitForService(service)) {
			std::cerr << "Service did not become available.\n";
			return 1;
		}
	}

	if (!ros::service::call(service, calibrate)) {
		std::cerr << "Service call `" << service << "' failed.\n";
		return 1;
	}

	std::cerr << "Camera calibration finished.\n";

	return 0;
}
