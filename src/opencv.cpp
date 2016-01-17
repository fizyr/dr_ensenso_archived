// Must include opencv version information before nxLib, so make it the first include.
#include <opencv2/core.hpp>
#include "opencv.hpp"
#include "util.hpp"

namespace dr {

cv::Mat toCvMat(NxLibItem const & item) {
	try {
		cv::Mat result;
		item.setBinaryData(result);
		return result;
	} catch (NxLibException const & e) {
		throw NxError(e);
	}
}

}
