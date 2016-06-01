#include <opencv2/opencv.hpp>

int main(int argc, char * * argv) {
	// This node is just for test
	cv::Mat image = cv::imread("/home/delftrobotics/raw_left_test2.png", CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<cv::Point2f> image_points;
	cv::Size pattern_size(7,7);

	cv::SimpleBlobDetector::Params params;
	params.filterByColor = false;
	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);

	bool found = cv::findCirclesGrid(image, pattern_size, image_points, cv::CALIB_CB_SYMMETRIC_GRID, blob_detector);

	cv::drawChessboardCorners(image, pattern_size, image_points, found);

	cv::imshow("draw", image);
	cv::waitKey();
}
