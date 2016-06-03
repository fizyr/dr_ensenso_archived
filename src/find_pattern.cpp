#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <ros/package.h>
#include <dr_pcl/3d_reconstruction.hpp>
#include <dr_pcl/pointcloud_tools.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace dr {

//! A function object that does nothing and can be used as an empty deleter for \c shared_ptr
struct null_deleter
{
	//! Function object result type
	typedef void result_type;
	/*!
	 * Does nothing
	 */
	template< typename T >
		void operator() (T*) const noexcept {}
};

template <typename T>
boost::shared_ptr<T> makeFakeShared(T & object) {
	return boost::shared_ptr<T>(&object, null_deleter{});
}

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

	bool found = cv::findCirclesGrid(image, pattern_size, image_points, cv::CALIB_CB_SYMMETRIC_GRID, blob_detector);

	cv::Mat draw;
	image.copyTo(draw);
	cv::drawChessboardCorners(draw, pattern_size, image_points, found);
	cv::imshow("draw",draw);
	cv::waitKey();

	return image_points;
}

pcl::PointCloud<pcl::PointXYZ> generateEnsensoCalibrationPattern() {
	pcl::PointCloud<pcl::PointXYZ> result;
	double pattern_size(7);
	double distance(0.01875);
	double center((pattern_size-1) * distance);
	for (size_t i=0; i<pattern_size; i++) {
		for (size_t j=0; j<pattern_size; j++) {
			double x(j*distance - center);
			double y(i*distance - center);
			double z(0);
			result.push_back(pcl::PointXYZ(x,y,z));
		}
	}
	return result;
}


} // namespace

int main(int argc, char * * argv) {
	(void) argc;
	(void) argv;

	cv::Mat left_image  = cv::imread(ros::package::getPath("dr_ensenso") + "/data/raw_left.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (left_image.empty()) {
		std::cerr << "Could not load image" << std::endl;
		return 1;
	}
	cv::Mat right_image = cv::imread(ros::package::getPath("dr_ensenso") + "/data/raw_right.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (right_image.empty()) {
		std::cerr << "Could not load image" << std::endl;
		return 1;
	}

	std::vector<cv::Point2f> left_points  = dr::findPattern(left_image);
	std::vector<cv::Point2f> right_points = dr::findPattern(right_image);

	if (left_points.size() == 0 || right_points.size() == 0) {
		std::cerr << "Pattern not detected" << std::endl;
		return 1;
	}

	sensor_msgs::CameraInfo camera_info_left, camera_info_right;
	std::string camera_name_left, camera_name_right;
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Left.yaml", camera_name_left, camera_info_left);
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Right.yaml", camera_name_right, camera_info_right);

	image_geometry::StereoCameraModel stereo_model;
	stereo_model.fromCameraInfo(camera_info_left, camera_info_right);

	pcl::PointCloud<pcl::PointXYZ> measured_pattern;
	for (size_t i=0; i<left_points.size(); i++) {
		cv::Point2d left_rectified  = stereo_model.left().rectifyPoint(left_points.at(i));
		cv::Point2d right_rectified = stereo_model.right().rectifyPoint(right_points.at(i));

		double depth(stereo_model.getZ(left_rectified.x-right_rectified.x));
		cv::Point2f focal_length(camera_info_left.P[0], camera_info_left.P[5]);
		cv::Point2f image_center(camera_info_left.P[2], camera_info_left.P[6]);

		// ToDo: in dr_pcl 3dreconstruction Correct for CX_Left - CX_Right (See image geometry) -right_.Tx() / (disparity - (left().cx() - right().cx()));

		float x, y, z;
		dr::get3dCoordinates(left_rectified, depth, focal_length, image_center, x, y, z);
		measured_pattern.push_back(pcl::PointXYZ(x, y, z));
	}

	pcl::PointCloud<pcl::PointXYZ> pattern = dr::generateEnsensoCalibrationPattern();

	Eigen::Isometry3d isometry = dr::findIsometry(
		dr::makeFakeShared(pattern),
		dr::makeFakeShared(measured_pattern)
	);
	std::cout << "Pattern pose: \n" << isometry.matrix() << std::endl;
}
