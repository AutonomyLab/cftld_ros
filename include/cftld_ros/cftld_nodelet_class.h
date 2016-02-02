#include <ros/node_handle.h>
#include <nodelet/nodelet.h>

namespace cftld_ros
{
class CFtldRosNodelet : public nodelet::Nodelet
{
public:
  CFtldRosNodelet();

private:
  virtual void onInit();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
}; // class CFtldRosNodelet
}  // namespace cftld_ros
