#pragma once

#include <vive_provider/vive.pb.h>

#include <Eigen/Geometry>

namespace vive_provider
{

Eigen::Vector3d getPos(const Vector3d & pos);

Eigen::Quaterniond getQuaternion(const Quaternion & q);

Eigen::Affine3d getWorldToTracker(const TrackerMsg & tracker);

}  // namespace vive_provider
