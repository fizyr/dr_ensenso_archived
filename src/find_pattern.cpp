#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>

namespace dr {

std::vector<cv::Point2f> findPattern(cv::Mat const & image) {
	std::vector<cv::Point2f> image_points;
	cv::Size pattern_size(7,7);

	cv::SimpleBlobDetector::Params params;
	params.filterByColor = false;
	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);

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

	cv::Point2d left_rectified = left_model.rectifyPoint(left_points.at(0));
	cv::Point2d right_rectified = left_model.rectifyPoint(right_points.at(0));

	double disparity = right_rectified.x - left_rectified.x;
	std::cout << "disparity: " << disparity << std::endl;
	double focus = (camera_info_left.P[0] + camera_info_left.P[5]) / 2;
	double baseline = -camera_info_right.P[3] / camera_info_left.P[0];
	std::cout << "focus: " << focus << std::endl;
	std::cout << "baseline: " << baseline << std::endl;
	double depth = (focus * baseline) / disparity;

	cv::Point3d ray = left_model.projectPixelTo3dRay(left_rectified);
	std::cout << "ray " << ray << std::endl;
	std::cout << "depth " << depth << std::endl;
	double norm = cv::norm(ray);
	cv::Point3d xyz = (depth / norm) * ray;

	std::cout << "xyz: '" << xyz << "'." << std::endl;

}
