// c++
#include <memory>
#include <string>

// ros
#include <ros/node_handle.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>

// cv
#include <opencv2/core/core.hpp>

// CFtld
#include "tld/TLD.h"
#include "main/Trajectory.h"
#include "main/Settings.h"
#include "cf_tracker.hpp"
#include "libconfig.h++"


namespace cftld_ros
{

enum tracking_state_t
{
  TRACKING_STATE_UNINITED = 0,
  TRACKING_STATE_INITED = 1,
  TRACKING_STATE_LOST = 2,
  TRACKING_STATE_TRACKING = 3,
  TRACKING_STATE_NUM
};

template<class T>
static void GetParam(const ros::NodeHandle& nh,
                     const std::string& param_name, T& var, const T& default_value)
{
  nh.param<T>(param_name, var, default_value);
  ROS_INFO_STREAM("[CFTLD] Param " << param_name << " : " << var);
}

class CFtldRosNodelet : public nodelet::Nodelet
{
public:
  CFtldRosNodelet();

protected:
  virtual void onInit();
  void ImageCallback(const sensor_msgs::ImageConstPtr& img_msg_ptr);
  void InitROICallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr);
  void ResetCallback(const std_msgs::EmptyConstPtr& empty_msg_ptr);
  void UpdateParams();
  bool InitCFtld(const std::string& cfg_file_path);

  cv::Mat frame_input_;
  cv::Mat frame_debug_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::shared_ptr<image_transport::ImageTransport> it_ptr_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_debug_image_;
  ros::Subscriber sub_roi_;
  ros::Subscriber sub_reset_;
  ros::Publisher pub_track_;

  // params
  std::string param_cfg_file_;
  double param_downsample_factor_;

  bool do_downsample_;
  bool enable_debug_image_;

  cv_bridge::CvImageConstPtr frame_input_cvptr_;
  cv_bridge::CvImage frame_debug_cvi_;

  // CFtld
  cftld_ros::tracking_state_t tracking_state_;
  libconfig::Config tld_cfg_;
  tld::Settings tld_settings_;
  std::shared_ptr<tld::TLD> tld_ptr_;
  std::shared_ptr<tld::Trajectory> trajectory_ptr_;
  bool param_show_output_;
  double param_tld_threshold_;
  int32_t pram_tld_seed_;

  uint32_t track_id_;

}; // class CFtldRosNodelet
}  // namespace cftld_ros
