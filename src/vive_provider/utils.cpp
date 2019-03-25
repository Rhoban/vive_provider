#include <vive_provider/utils.h>
#include <rhoban_utils/util.h>

namespace vive_provider
{

Eigen::Vector3d getPos(const Vector3d & pos)
{
  return Eigen::Vector3d(pos.x(), pos.y(), pos.z());
}

Eigen::Quaterniond getQuaternion(const Quaternion & q)
{
  return Eigen::Quaterniond(q.qw(), q.qx(), q.qy(), q.qz());
}

Eigen::Affine3d getWorldToTracker(const TrackerMsg & tracker)
{
  if (!tracker.has_pos())
  {
    throw std::logic_error(DEBUG_INFO + "tracker has no pos provided");
  }
  if (!tracker.has_orientation())
  {
    throw std::logic_error(DEBUG_INFO + "tracker has no pos provided");
  }
  Eigen::Vector3d pos = getPos(tracker.pos());
  Eigen::Quaterniond q = getQuaternion(tracker.orientation());
  return Eigen::Translation3d(pos) * Eigen::Affine3d(q);
  
}

}  // namespace vive_provider
