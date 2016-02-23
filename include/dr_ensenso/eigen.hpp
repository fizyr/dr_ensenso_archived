#pragma once
#include <Eigen/Dense>
#include <ensenso/nxLib.h>
#include <cstdint>
#include "util.hpp"

namespace dr {

/// Convert an NxLibItem holding a 3D vector to an Eigen::Vector3d.
Eigen::Vector3d toEigenVector(NxLibItem const & item);

/// Convert an NxLibItem holding a 3D vector to an Eigen::Translation3d.
Eigen::Translation3d toEigenTranslation(NxLibItem const & item);

/// Convert an NxLibItem holding a rotation to an Eigen::AngleAxisd.
Eigen::AngleAxisd toEigenRotation(NxLibItem const & item);

/// Convert an NxLibItem holding a pose or transformation to an Eigen::Isometry3d.
Eigen::Isometry3d toEigenIsometry(NxLibItem const & item);

template <std::size_t rows, std::size_t cols>
Eigen::Matrix<double, rows, cols> toEigenMatrix(NxLibItem const & item) {
	Eigen::Matrix<double, rows, cols> result;

	for (std::size_t row = 0; row < rows; ++row) {
		for (std::size_t col = 0; col < cols; ++col) {
			result(row, col) = getNx<double>(item[col][row]);
		}
	}

	return result;
}

}
