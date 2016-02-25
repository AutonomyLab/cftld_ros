#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <cftld_ros/Track.h>

class FakeTracker
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber sub_roi_;
  ros::Publisher pub_track_;

  cftld_ros::Track track_msg_;

  ros::Time track_recv_time_;
  sensor_msgs::RegionOfInterestConstPtr roi_ptr_;

  void ROICallback(const sensor_msgs::RegionOfInterestConstPtr& roi_ptr)
  {
    track_recv_time_ = ros::Time::now();
    roi_ptr_ = roi_ptr;
  }

public:
  FakeTracker(ros::NodeHandle nh)
    : nh_(nh),
      nh_priv_("~"),
      sub_roi_(nh_.subscribe("roi", 30, &FakeTracker::ROICallback, this)),
      pub_track_(nh_.advertise<cftld_ros::Track>("track", 30)),
      track_recv_time_(0.0)
  {}

  void Spin()
  {
    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
      track_msg_.header.stamp = ros::Time::now();
      track_msg_.header.frame_id = "";


      const bool is_valid_roi = ((ros::Time::now() - track_recv_time_).toSec() < 1.0) &&
          roi_ptr_ &&
          (roi_ptr_->x_offset > 0) && (roi_ptr_->y_offset > 0) && (roi_ptr_->width > 5) && (roi_ptr_->height > 5) &&
          (roi_ptr_->width < 640) && (roi_ptr_->height < 368);

      if (!is_valid_roi)
      {
        track_msg_.uid = 1;
        track_msg_.status = cftld_ros::Track::STATUS_LOST;
        track_msg_.confidence = 0.0;
        track_msg_.roi.x_offset = 0;
        track_msg_.roi.y_offset = 0;
        track_msg_.roi.width = 0;
        track_msg_.roi.height = 0;
      }
      else
      {
        track_msg_.uid = 1;
        track_msg_.status = cftld_ros::Track::STATUS_TRACKING;
        track_msg_.confidence = 1.0;
        track_msg_.roi = *roi_ptr_;
      }
      pub_track_.publish(track_msg_);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fake_tracker");
  ros::NodeHandle nh;

  ROS_INFO("[FAK] Starting Fake Tracker ...");
  FakeTracker fake_tracker(nh);

  fake_tracker.Spin();

  return 0;
}
