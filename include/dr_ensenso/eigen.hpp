#pragma once
#include <Eigen/Dense>
#include <nxLib.h>

namespace dr {

/// Convert an NxLibItem holding a 3D vector to an Eigen::Vector3d.
Eigen::Vector3d toEigenVector(NxLibItem const & item);

/// Convert an NxLibItem holding a 3D vector to an Eigen::Translation3d.
Eigen::Translation3d toEigenTranslation(NxLibItem const & item);

/// Convert an NxLibItem holding a rotation to an Eigen::AngleAxisd.
Eigen::AngleAxisd toEigenRotation(NxLibItem const & item);

/// Convert an NxLibItem holding a pose or transformation to an Eigen::Isometry3d.
Eigen::Isometry3d toEigenIsometry(NxLibItem const & item);

}
