#pragma once

#include <ensenso/nxLib.h>
#include <opencv2/core/core.hpp>

namespace dr {

/// Convert an NxLibItem to a cv::Mat.
/**
 * \throw NxError on failure.
 */
cv::Mat toCvMat(NxLibItem const & item);

}
