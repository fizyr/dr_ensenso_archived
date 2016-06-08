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

#include "ensenso.hpp"
#include "eigen.hpp"
#include "opencv.hpp"
#include "util.hpp"

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
	double center(0.5*(pattern_size-1) * distance);
	for (size_t i=0; i<pattern_size; i++) {
		for (size_t j=0; j<pattern_size; j++) {
			double x(i*distance - center);
			double y((-1)*(j*distance - center));
			double z(0);
			result.push_back(pcl::PointXYZ(x,y,z));
		}
	}
	return result;
}

void getEnsensoPoints(std::vector<cv::Point2f> & left_points, std::vector<cv::Point2f> & right_points, size_t pattern_size) {
	NxLibCommand get_pattern_buffers(cmdGetPatternBuffers);
	dr::executeNx(get_pattern_buffers);
	for (size_t i = 0; i < pattern_size*pattern_size; i++) {
		left_points.push_back(cv::Point2f(
				get_pattern_buffers.result()[itmStereo][0][itmPoints][0][i][0].asDouble(),
				get_pattern_buffers.result()[itmStereo][0][itmPoints][0][i][1].asDouble()
		));
		right_points.push_back(cv::Point2f(
			get_pattern_buffers.result()[itmStereo][0][itmPoints][1][i][0].asDouble(),
			get_pattern_buffers.result()[itmStereo][0][itmPoints][1][i][1].asDouble()
		));
	}
}

Eigen::Isometry3d getPatternPose(
		std::vector<cv::Point2f> const & left_points,
		std::vector<cv::Point2f> const & right_points,
		std::vector<cv::Point2f> & left_rectified,
		std::vector<cv::Point2f> & right_rectified,
		pcl::PointCloud<pcl::PointXYZ> & measured_pattern
) {
	sensor_msgs::CameraInfo camera_info_left, camera_info_right;
	std::string camera_name_left, camera_name_right;
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Left.yaml", camera_name_left, camera_info_left);
	camera_calibration_parsers::readCalibration(ros::package::getPath("dr_ensenso") + "/data/Right.yaml", camera_name_right, camera_info_right);

	image_geometry::StereoCameraModel stereo_model;
	stereo_model.fromCameraInfo(camera_info_left, camera_info_right);

	for (size_t i=0; i<left_points.size(); i++) {
		left_rectified.push_back(stereo_model.left().rectifyPoint(left_points.at(i)));
		right_rectified.push_back(stereo_model.right().rectifyPoint(right_points.at(i)));

		//double depth(stereo_model.getZ(left_rectified.x-right_rectified.x));
		//cv::Point2f focal_length(camera_info_left.P[0], camera_info_left.P[5]);
		//cv::Point2f image_center(camera_info_left.P[2], camera_info_left.P[6]);
		// ToDo: in dr_pcl 3dreconstruction Correct for CX_Left - CX_Right (See image geometry) -right_.Tx() / (disparity - (left().cx() - right().cx()));
		//float x, y, z;
		//dr::get3dCoordinates(left_rectified, depth, focal_length, image_center, x, y, z);

		cv::Point3d point;
		stereo_model.projectDisparityTo3d(left_rectified.back(), left_rectified.back().x-right_rectified.back().x, point);

		measured_pattern.push_back(pcl::PointXYZ(point.x, point.y, point.z));
	}

	// Compare raw, undistorted and rect image points
	std::vector<cv::Point2f> left_points_undistorted;
	std::vector<cv::Point2f> left_points_opencv;
	cv::undistortPoints(left_points, left_points_undistorted, cv::Mat(stereo_model.left().intrinsicMatrix()), cv::Mat(stereo_model.left().distortionCoeffs()));
	cv::undistortPoints(left_points, left_points_opencv, cv::Mat(stereo_model.left().intrinsicMatrix()), cv::Mat(stereo_model.left().distortionCoeffs()), cv::Mat(stereo_model.left().rotationMatrix()));
	std::cout << "D:\n" << cv::Mat(stereo_model.left().distortionCoeffs()) << std::endl;
	std::cout << "K:\n" << cv::Mat(stereo_model.left().intrinsicMatrix()) << std::endl;
	std::cout << "Left points raw:\n" << left_points << std::endl;
	std::cout << "Left points undistorted:\n" << left_points_undistorted << std::endl;
	std::cout << "Left points undistorted and rectified (opencv):\n" << left_points_opencv << std::endl;
	std::cout << "Left points undistorted and rectified (image geometry):\n" << left_rectified << std::endl;

	pcl::PointCloud<pcl::PointXYZ> pattern = dr::generateEnsensoCalibrationPattern();

	Eigen::Isometry3d isometry = dr::findIsometry(
		dr::makeFakeShared(pattern),
		dr::makeFakeShared(measured_pattern)
	);
	return isometry;
}

void drawPattern(cv::Mat const & image, std::vector<cv::Point2f> const & points) {
	cv::Mat draw;
	image.copyTo(draw);
	cv::drawChessboardCorners(draw, cv::Size(7,7), points, true);
	cv::imshow("draw", draw);
	cv::waitKey();
}

/// Set workspace calibration to the left camera origin
void setWorkspaceToLeftCamera() {
	NxLibCommand command(cmdCalibrateWorkspace);
	executeNx(command);
}

// Get the pose of the ensenso calibration pattern
Eigen::Isometry3d getPatternPose(Ensenso & ensenso) {
	setWorkspaceToLeftCamera();
	ensenso.recordCalibrationPattern();

	NxLibCommand command(cmdEstimatePatternPose);
	executeNx(command);
	Eigen::Isometry3d pose = toEigenIsometry(command.result()[itmPatterns][0][itmPatternPose]);
	return pose;
}

Eigen::Matrix<double, 4, 4> getReprojectionMatrix(NxLibItem const & item) {
	Eigen::Matrix<double,4,4> reprojection_matrix;
	for (size_t i=0; i<4; i++) {
		for (size_t j=0; j<4; j++) {
			reprojection_matrix(j,i) = item[i][j].asDouble();
		}
	}
	return reprojection_matrix;
}

Eigen::Isometry3d getPatternPoseUsingReprojection(
	Ensenso & ensenso,
	std::vector<cv::Point2f> const & left_rectified_ensenso,
	std::vector<cv::Point2f> const & right_rectified_ensenso
) {
	Eigen::Matrix4d Q = dr::getReprojectionMatrix(ensenso.native()[itmCalibration][itmStereo][itmReprojection]);

	pcl::PointCloud<pcl::PointXYZ> measured_pattern;
	for (size_t i=0; i<left_rectified_ensenso.size(); i++) {
		Eigen::Vector4d vector(left_rectified_ensenso.at(i).x, left_rectified_ensenso.at(i).y, left_rectified_ensenso.at(i).x - right_rectified_ensenso.at(i).x, 1);
		Eigen::Vector4d result(Q*vector);
		measured_pattern.push_back( pcl::PointXYZ(result(0)/result(3), result(1)/result(3), result(2)/result(3) ) );
	}

	pcl::PointCloud<pcl::PointXYZ> pattern = generateEnsensoCalibrationPattern();
	return dr::findIsometry(
		dr::makeFakeShared(pattern),
		dr::makeFakeShared(measured_pattern)
	);
}

void rectify() {
	NxLibCommand command(cmdRectifyImages);
	executeNx(command);
}

void drawRectifiedPoints(
	Ensenso & ensenso,
	std::vector<cv::Point2f> const & left_rectified_dr,
	std::vector<cv::Point2f> const & right_rectified_dr,
	std::vector<cv::Point2f> const & left_rectified_ensenso,
	std::vector<cv::Point2f> const & right_rectified_ensenso
) {
	cv::Mat left_rect =  dr::toCvMat(ensenso.native()[itmImages][itmRectified][itmLeft]);
	drawPattern(left_rect, left_rectified_dr);
	drawPattern(left_rect, left_rectified_ensenso);

	cv::Mat right_rect = dr::toCvMat(ensenso.native()[itmImages][itmRectified][itmRight]);
	drawPattern(right_rect, right_rectified_dr);
	drawPattern(right_rect, right_rectified_ensenso);
}

/// Backproject from cloud of xyz points in world coordinates to u,v coordinates in rectified image using reprojection matrix Q
std::vector<cv::Point2f> backProject(Eigen::Matrix4d const & Q, pcl::PointCloud<pcl::PointXYZ> const & cloud){
	std::vector<cv::Point2f> rectified_image_points;
	for (size_t i=0; i<cloud.size(); i++) {
		Eigen::Vector4d point(cloud.at(i).x, cloud.at(i).y, cloud.at(i).z, 1);
		Eigen::Vector4d result(Q.inverse() * point);
		result = result / result(3);
		// result(2) contains the disparity and result(3) is the homogeneous part.
		cv::Point2f rect_image_point(result(0), result(1));
		rectified_image_points.push_back(rect_image_point);
	}
	return rectified_image_points;
}

} // namespace

int main(int argc, char * * argv) {
	(void) argc;
	(void) argv;

	dr::Ensenso ensenso(false);
	dr::setNx(ensenso.native()[itmParameters][itmCapture][itmProjector], false);
	ensenso.retrieve(true,5000,true,false);
	cv::Mat left_image = dr::toCvMat(ensenso.native()[itmImages][itmRaw][itmLeft]);
	cv::Mat right_image = dr::toCvMat(ensenso.native()[itmImages][itmRaw][itmRight]);

	// Get left and right image points as processed by ensenso sdk
	ensenso.recordCalibrationPattern();
	std::vector<cv::Point2f> left_points_ensenso, right_points_ensenso;
	dr::getEnsensoPoints(left_points_ensenso, right_points_ensenso, 7);
	dr::drawPattern(left_image, left_points_ensenso);
	dr::drawPattern(right_image, right_points_ensenso);

	// Get left and right points as processed by dr (opencv) blob detection
	std::vector<cv::Point2f> left_points_dr  = dr::findPattern(left_image);
	std::vector<cv::Point2f> right_points_dr = dr::findPattern(right_image);

	std::vector<cv::Point2f> left_rectified_dr;
	std::vector<cv::Point2f> right_rectified_dr;
	pcl::PointCloud<pcl::PointXYZ> measured_pattern_dr;
	Eigen::Isometry3d isometry_dr = dr::getPatternPose(left_points_dr, right_points_dr, left_rectified_dr, right_rectified_dr, measured_pattern_dr);
	std::cout << "DR Pattern pose with open source ray tracing: \n" << isometry_dr.matrix() << "\n" << std::endl;

	std::vector<cv::Point2f> left_rectified_ensenso;
	std::vector<cv::Point2f> right_rectified_ensenso;
	pcl::PointCloud<pcl::PointXYZ> measured_pattern_ensenso;
	Eigen::Isometry3d isometry_dr_ensenso = dr::getPatternPose(left_points_ensenso, right_points_ensenso, left_rectified_ensenso, right_rectified_ensenso, measured_pattern_ensenso);
	std::cout << "Ensenso Pattern pose with open source ray tracing: \n " << isometry_dr_ensenso.matrix() << "\n" << std::endl;

	// Draw undistorted rectified points on the rectified image
	dr::rectify();
	dr::drawRectifiedPoints(ensenso, left_rectified_dr, right_rectified_ensenso, left_rectified_dr, right_rectified_dr);

	Eigen::Isometry3d isometry_ensenso = getPatternPose(ensenso);
	std::cout << "Ensenso Pattern pose with Ensenso ray tracing: \n " << isometry_ensenso.matrix() << "\n" << std::endl;

	// Use reprojection matrix (4x4) from Ensenso directly to calculate point location with ensenso points
	Eigen::Isometry3d pattern_pose_using_reprojection = dr::getPatternPoseUsingReprojection(ensenso, left_rectified_ensenso, right_rectified_ensenso);
	std::cout << "Ensenso pattern pose using Q matrix and ensenso image points:\n" << pattern_pose_using_reprojection.matrix() << "\n" << std::endl;
}
