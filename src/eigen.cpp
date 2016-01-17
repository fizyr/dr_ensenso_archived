#include "eigen.hpp"
#include "util.hpp"

namespace dr {

Eigen::Vector3d toEigenVector(NxLibItem const & item) {
	return Eigen::Vector3d{getNx<double>(item[0]), getNx<double>(item[1]), getNx<double>(item[2])};
}

Eigen::Translation3d toEigenTranslation(NxLibItem const & item) {
	return Eigen::Translation3d{toEigenVector(item)};
}

Eigen::AngleAxisd toEigenRotation(NxLibItem const & item) {
	return Eigen::AngleAxisd{getNx<double>(item[itmAngle]), toEigenVector(item[itmAxis])};
}

Eigen::Isometry3d toEigenIsometry(NxLibItem const & item) {
	return toEigenTranslation(item[itmTranslation]) * toEigenRotation(item[itmRotation]);
}

}
