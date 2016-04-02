// Must include opencv version information before nxLib, so make it the first include.
#include <opencv2/opencv.hpp>
#include "opencv.hpp"
#include "util.hpp"

namespace dr {

cv::Mat toCvMat(NxLibItem const & item, std::string const & what) {
	int error = 0;
	cv::Mat result;
	item.getBinaryData(&error, result, nullptr);
	if (error) throw NxError(item, error, what);

	// convert RGB output from camera to OpenCV standard (BGR)
	if (result.channels() == 3) {
		cv::cvtColor(result, result, cv::COLOR_RGB2BGR);
	}
	return result;
}

}
