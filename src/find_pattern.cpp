#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>
#include <dr_pcl/3d_reconstruction.hpp>

namespace dr {

std::vector<cv::Point2f> findPattern(cv::Mat const & image) {
	std::vector<cv::Point2f> image_points;
	cv::Size pattern_size(7,7);

	cv::SimpleBlobDetector::Params params;
	params.filterByColor = false;

#if CV_MAJOR_VERSION < 3
	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
#else
	cv::Ptr<cv::FeatureDetector> blob_detector = cv::SimpleBlobDetector::create(params);
#endif

	cv::findCirclesGrid(image, pattern_size, image_points, cv::CALIB_CB_SYMMETRIC_GRID, blob_detector);
	return image_points;
}

} // namespace

int main(int argc, char * * argv) {
	(void) argc;
	(void) argv;

	cv::Mat left_image  = cv::imread(ros::package::getPath("dr_ensenso") + "/data/raw_left.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat right_image = cv::imread(ros::package::getPath("dr_ensenso") + "/data/raw_right.png", CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<cv::Point2f> left_points  = dr::findPattern(left_image);
	std::vector<cv::Point2f> right_points = dr::findPattern(right_image);

	sensor_msgs::CameraInfo camera_info_left, camera_info_right;
	std::string camera_name_left, camera_name_right;
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Left.yaml", camera_name_left, camera_info_left);
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Right.yaml", camera_name_right, camera_info_right);

	image_geometry::PinholeCameraModel left_model;
	left_model.fromCameraInfo(camera_info_left);

	image_geometry::PinholeCameraModel right_model;
	right_model.fromCameraInfo(camera_info_right);

	cv::Point2d left_rectified  = left_model.rectifyPoint(left_points.at(1));
	cv::Point2d right_rectified = right_model.rectifyPoint(right_points.at(1));

	double baseline(-camera_info_right.P[3] / camera_info_left.P[0]);
	float x,y,z;
	dr::get3dCoordinates(
		cv::Point2f(left_rectified.x, left_rectified.y),            // Coordinates of the point in the left image
		cv::Point2f(right_rectified.x, right_rectified.y),          // Coordinates of the point in the right image
		baseline,                                                   // Camera's baseline
		cv::Point2f(camera_info_left.P[0], camera_info_left.P[5]),  // Camera's focal length
		cv::Point2f(camera_info_left.P[2], camera_info_left.P[6]),  // Camera's image center
		x,                                                          // Output point's X coordinate
		y,                                                          // Output point's Y coordinate
		z                                                           // Output point's Z coordinate
	);

	std::cout << "xyz: " << cv::Point3d(x,y,z) << std::endl;

}
