#include <pluginlib/class_list_macros.h>

#include "cftld_ros/cftld_nodelet_class.h"

namespace cftld_ros
{

CFtldRosNodelet::CFtldRosNodelet()
{
  //
}

void CFtldRosNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
}

}  // namespace cftld_ros

PLUGINLIB_DECLARE_CLASS(cftld_ros, CFtldRosNodelet, cftld_ros::CFtldRosNodelet, nodelet::Nodelet);
