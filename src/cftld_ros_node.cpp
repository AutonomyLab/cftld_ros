#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cftld_ros_node", ros::init_options::NoSigintHandler);
  nodelet::Loader nll;

  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  const std::string nl_name = ros::this_node::getName();
  nll.load(nl_name, "cftld_ros/CFtldRosNodelet", remap, nargv);

  const std::vector<std::string>& loaded_nodelets = nll.listLoadedNodelets();
  if (std::find(loaded_nodelets.begin(),
                loaded_nodelets.end(),
                nl_name) == loaded_nodelets.end())
  {
    // Nodelet OnInit() failed
    ROS_FATAL("cftld_ros nodelet failed to load.");
    return 1;
  }

  // It reaches here when OnInit() succeeds
  ROS_INFO("cftld_ros nodelet loaded.");
  ros::spin();
  return 0;
}
