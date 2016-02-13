// c++
#include "cftld_ros/cftld_nodelet_class.h"

// ros
#include <pluginlib/class_list_macros.h>
#include "cv_bridge/cv_bridge.h"

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// CFtld
#include "tld/DetectorCascade.h"

// Misc
#include "cftld_ros/benchmarker.hpp"

namespace cftld_ros
{

CFtldRosNodelet::CFtldRosNodelet()
  : param_tld_cfgfile_(""),
    param_downsample_factor_(1.0),
    do_downsample_(false),
    tracking_state_(TRACKING_STATE_UNINITED),
    tld_ptr_(new tld::TLD()),
    track_id_(0)
{
  //
}

void CFtldRosNodelet::UpdateParams()
{
   GetParam<bool>(private_nh_, "enable_debug_image", enable_debug_image_, false);
   GetParam<std::string>(private_nh_, "tld_config_file", param_tld_cfgfile_, std::string(""));
   GetParam<double>(private_nh_, "downsample_factor", param_downsample_factor_, 1.0);
}

bool CFtldRosNodelet::InitCFtld(const std::string &cfg_file_path)
{
  if (!cfg_file_path.empty())
  {
    NODELET_INFO_STREAM("[CFTLD] Config file " << cfg_file_path);
    try
    {
      tld_cfg_.readFile(cfg_file_path.c_str());
    }
    catch (const libconfig::FileIOException &fioex)
    {
      NODELET_ERROR_STREAM("[CFTLD] I/O error while reading config file: " << fioex.what());
      return false;
    }
    catch (const libconfig::ParseException &pex)
    {
      NODELET_ERROR_STREAM("[CFTLD] ConfigFile parse error" << pex.what());
      return false;
    }
    // If cfg file is valid and succesfully parsed, overwrite default settings
    // Not all are supported
    tld_cfg_.lookupValue("acq.startFrame", tld_settings_.m_startFrame);
    tld_cfg_.lookupValue("acq.lastFrame", tld_settings_.m_lastFrame);
    tld_cfg_.lookupValue("detector.useProportionalShift", tld_settings_.m_useProportionalShift);
    tld_cfg_.lookupValue("detector.proportionalShift", tld_settings_.m_proportionalShift);
    tld_cfg_.lookupValue("detector.minScale", tld_settings_.m_minScale);
    tld_cfg_.lookupValue("detector.maxScale", tld_settings_.m_maxScale);
    tld_cfg_.lookupValue("detector.minSize", tld_settings_.m_minSize);
    tld_cfg_.lookupValue("detector.numTrees", tld_settings_.m_numTrees);
    tld_cfg_.lookupValue("detector.numFeatures", tld_settings_.m_numFeatures);
    tld_cfg_.lookupValue("showOutput", tld_settings_.m_showOutput);
    tld_cfg_.lookupValue("trajectory", tld_settings_.m_trajectory);
    tld_cfg_.lookupValue("printResults", tld_settings_.m_printResults);
    tld_cfg_.lookupValue("printTiming", tld_settings_.m_printTiming);
    tld_cfg_.lookupValue("learningEnabled", tld_settings_.m_learningEnabled);
    tld_cfg_.lookupValue("trackerEnabled", tld_settings_.m_trackerEnabled);
    tld_cfg_.lookupValue("detectorEnabled", tld_settings_.m_detectorEnabled);
    tld_cfg_.lookupValue("detector.varianceFilterEnabled", tld_settings_.m_varianceFilterEnabled);
    tld_cfg_.lookupValue("detector.ensembleClassifierEnabled", tld_settings_.m_ensembleClassifierEnabled);
    tld_cfg_.lookupValue("detector.nnClassifierEnabled", tld_settings_.m_nnClassifierEnabled);
    tld_cfg_.lookupValue("useDsstTracker", tld_settings_.m_useDsstTracker);
    tld_cfg_.lookupValue("selectManually", tld_settings_.m_selectManually);
    tld_cfg_.lookupValue("saveDir", tld_settings_.m_outputDir);
    tld_cfg_.lookupValue("threshold", tld_settings_.m_threshold);
    tld_cfg_.lookupValue("showNotConfident", tld_settings_.m_showNotConfident);
    tld_cfg_.lookupValue("showDetections", tld_settings_.m_showDetections);
    tld_cfg_.lookupValue("alternating", tld_settings_.m_alternating);
    tld_cfg_.lookupValue("seed", tld_settings_.m_seed);
  }
  else
  {
    NODELET_WARN("[CFTLD] Path to the config file is empty. Using default settings ....");
  }

  // Initialize TLD
  SetTLDParam("tld.trackerEnabled", tld_ptr_->trackerEnabled, tld_settings_.m_trackerEnabled);
  SetTLDParam("tld.detectorEnabled", tld_ptr_->detectorEnabled, tld_settings_.m_detectorEnabled);
  tld_ptr_->init(tld_settings_.m_useDsstTracker);
  SetTLDParam("tld.alternating", tld_ptr_->alternating, tld_settings_.m_alternating);
  SetTLDParam("tld.learningEnabled", tld_ptr_->learningEnabled, tld_settings_.m_learningEnabled);
  SetTLDParam("tld.seed", tld_ptr_->seed, tld_settings_.m_seed);

//  main->showOutput = m_settings.m_showOutput;
//  main->showTrajectory = (m_settings.m_trajectory) ? true : false;
//  main->trajectoryLength = m_settings.m_trajectory;
//  main->printResults = (m_settings.m_printResults.empty()) ? NULL : m_settings.m_printResults.c_str();
//  main->saveDir = (m_settings.m_outputDir.empty()) ? NULL : m_settings.m_outputDir.c_str();
//  main->threshold = m_settings.m_threshold;
//  main->showNotConfident = m_settings.m_showNotConfident;


//  main->selectManually = m_settings.m_selectManually;
//  main->seed = m_settings.m_seed;

  tld::DetectorCascade *detectorCascade = tld_ptr_->detectorCascade;
  ROS_ASSERT(detectorCascade);

  SetTLDParam("tld.detectorCascade.varianceFilter.enabled", detectorCascade->varianceFilter->enabled, tld_settings_.m_varianceFilterEnabled);
  SetTLDParam("tld.detectorCascade.ensembleClassifier.enabled", detectorCascade->ensembleClassifier->enabled, tld_settings_.m_ensembleClassifierEnabled);
  SetTLDParam("tld.detectorCascade.nnClassifier.enabled", detectorCascade->nnClassifier->enabled, tld_settings_.m_nnClassifierEnabled);
  SetTLDParam("tld.detectorCascade.nnClassifier.thetaTP", detectorCascade->nnClassifier->thetaTP, tld_settings_.m_thetaP);
  SetTLDParam("tld.detectorCascade.nnClassifier.thetaFP", detectorCascade->nnClassifier->thetaFP, tld_settings_.m_thetaN);

  // classifier
  SetTLDParam("tld.detectorCascade.useShift", detectorCascade->useShift, tld_settings_.m_useProportionalShift);
  SetTLDParam("tld.detectorCascade.shift", detectorCascade->shift, tld_settings_.m_proportionalShift);
  SetTLDParam("tld.detectorCascade.minScale", detectorCascade->minScale, tld_settings_.m_minScale);
  SetTLDParam("tld.detectorCascade.maxScale", detectorCascade->maxScale, tld_settings_.m_maxScale);
  SetTLDParam("tld.detectorCascade.minSize", detectorCascade->minSize, tld_settings_.m_minSize);
  SetTLDParam("tld.detectorCascade.numTrees", detectorCascade->numTrees, tld_settings_.m_numTrees);
  SetTLDParam("tld.detectorCascade.numFeatures", detectorCascade->numFeatures, tld_settings_.m_numFeatures);

  return true;
}

void CFtldRosNodelet::ResetCallback(const std_msgs::EmptyConstPtr& empty_msg_ptr)
{
  NODELET_WARN("[CFTLD] Reset requested");

  if (tld_ptr_)
  {
    NODELET_INFO("[CFTLD] Releasing the tracker");
    tld_ptr_->release();
  }

  tracking_state_ = TRACKING_STATE_UNINITED;
  track_id_ = 0;
}

void CFtldRosNodelet::InitROICallback(const sensor_msgs::RegionOfInterestConstPtr &roi_msg_ptr)
{
  NODELET_INFO_STREAM("[CFTLD] Initialize ROI requested [x,y,w,h]: "
                      << roi_msg_ptr->x_offset << " , "
                      << roi_msg_ptr->y_offset << " , "
                      << roi_msg_ptr->width << " , "
                      << roi_msg_ptr->height);

  if (!tld_ptr_)
  {
    NODELET_ERROR("[CFTLD] Tracker object is NULL");
    return;
  }

  if (frame_input_.empty())
  {
    NODELET_ERROR("[CFTLD] No input frame has been received or input frame is empty!");
    return;
  }

  // Bad API design of TLD
  cv::Rect bb(roi_msg_ptr->x_offset, roi_msg_ptr->y_offset,
              roi_msg_ptr->width, roi_msg_ptr->height);

  if (bb.area() < 10)
  {
    NODELET_ERROR("[CFTLD] The input rect's area is less than 10");
    return;
  }

  tld_ptr_->selectObject(frame_input_, &bb);
  tracking_state_ = TRACKING_STATE_INITED;
  track_id_++;
}

void CFtldRosNodelet::ImageCallback(const sensor_msgs::ImageConstPtr &img_msg_ptr)
{
  util::StepBenchmarker::GetInstance().reset();

  if (!tld_ptr_)
  {
    // This is the only case that we do not publish any tracking data,
    // this should actually never happen during healthy lifecycle of this nodelet
    NODELET_FATAL("[CFTLD] Tracker is NULL");
    return;
  }
  try
  {
    if (sensor_msgs::image_encodings::isColor(img_msg_ptr->encoding))
    {
      NODELET_WARN_ONCE("[CFTLD] Input image is BGR8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      NODELET_WARN_ONCE("[CFTLD] Input image is MONO8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::MONO8);
    }

    frame_input_ = frame_input_cvptr_->image;

    TICK("[CFTLD] Frame Copy");

    if (do_downsample_)
    {
      cv::resize(frame_input_, frame_input_, cv::Size(0, 0),
                 param_downsample_factor_, param_downsample_factor_, cv::INTER_CUBIC);
      TICK("[CFTLD] Downsample");
    }

    if (frame_input_.channels() == 1)
    {
      cv::cvtColor(frame_input_, frame_input_, cv::COLOR_GRAY2BGR);
      TICK("[CFTLD] Convert to BGR");
    }

    cftld_ros::TrackPtr track_msg_ptr(new cftld_ros::Track);
    track_msg_ptr->header = img_msg_ptr->header;
    track_msg_ptr->status = cftld_ros::Track::STATUS_UNKNOWN;
    track_msg_ptr->roi.x_offset = 0;
    track_msg_ptr->roi.y_offset = 0;
    track_msg_ptr->roi.width = 0;
    track_msg_ptr->roi.height = 0;
    track_msg_ptr->uid = 0;
    track_msg_ptr->confidence = 0.0f;

    if (tracking_state_ == TRACKING_STATE_UNINITED)
    {
      NODELET_INFO_THROTTLE(10, "[CFTLD] The tracker has not yet been initialized.");
      track_msg_ptr->status = cftld_ros::Track::STATUS_UNKNOWN;
    }
    else
    {
      NODELET_DEBUG_STREAM("TLD Tracking Valid: " << tld_ptr_->isTrackerValid);
      NODELET_DEBUG_STREAM("TLD Tracking Confidence: " << tld_ptr_->currConf);
      NODELET_DEBUG_STREAM("TLD Tracking Enabled: " << tld_ptr_->trackerEnabled);
      NODELET_DEBUG_STREAM("TLD Learning Enabled: " << tld_ptr_->learningEnabled);
      NODELET_DEBUG_STREAM("TLD Detection Enabled: " << tld_ptr_->detectorEnabled);
      NODELET_DEBUG_STREAM("TLD Valid BB: " << (tld_ptr_->currBB != NULL));

      tld_ptr_->processImage(frame_input_);
      TICK("[CFTLD] TLD processImage");
      if (tld_ptr_->currBB && tld_ptr_->isTrackerValid)
      {
        track_msg_ptr->status = cftld_ros::Track::STATUS_TRACKING;
        const cv::Rect& bb = *(tld_ptr_->currBB);
        track_msg_ptr->roi.x_offset = bb.x;
        track_msg_ptr->roi.y_offset = bb.y;
        track_msg_ptr->roi.width = bb.width;
        track_msg_ptr->roi.height = bb.height;
        track_msg_ptr->uid = track_id_;
        track_msg_ptr->confidence = tld_ptr_->currConf;
        NODELET_DEBUG_STREAM("[CFTLD] Tracking: " << bb);
      }
      else
      {
        // When lost, except for uid, we send default values (0)
        track_msg_ptr->status = cftld_ros::Track::STATUS_LOST;
        track_msg_ptr->uid = track_id_;
      }
    }

    pub_track_.publish(track_msg_ptr);
    TICK("[CFTLD] Publish results");

    if (enable_debug_image_ && (pub_debug_image_.getNumSubscribers() > 0))
    {
      frame_debug_ = frame_input_.clone();
      if (tld_ptr_ && tld_ptr_->currBB)
      {
        //cv::rectangle(frame_debug_, *(tld_ptr_->currBB), CV_RGB(100, 0, 0), 2);
        cv::rectangle(frame_debug_, *(tld_ptr_->currBB), CV_RGB(100, 0, 0), 2);
      }
      else
      {
        ; // TODO: Add debug text
      }

      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(img_msg_ptr->header,
                                                               sensor_msgs::image_encodings::BGR8,
                                                               frame_debug_).toImageMsg();

      pub_debug_image_.publish(debug_img_msg);
      TICK("[CFTLD] Debug Image");
    }

    NODELET_DEBUG_STREAM("Benchmark:\n" << util::StepBenchmarker::GetInstance().getstr());
  }
  catch (const cv_bridge::Exception& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] cv_bridge exception: " << e.what());
  }
  catch (const ros::Exception& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] ROS exception: " << e.what());
  }
  catch (const std::runtime_error& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] runtime exception: " << e.what());
  }
}

void CFtldRosNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  UpdateParams();

  NODELET_INFO_STREAM("[CFTLD] Initializing CFtld from " << param_tld_cfgfile_);
  if (!InitCFtld(param_tld_cfgfile_))
  {
    NODELET_ERROR_STREAM("[CFTLD] Initialzing CFtld failed!");
  }

  it_ptr_ = std::make_shared<image_transport::ImageTransport>(nh_);

  // In Nodelet version, the default transport hint does not use Nodelet's local and
  // private Nodehandles
  image_transport::TransportHints it_th("raw", ros::TransportHints(), private_nh_);
  sub_image_ = it_ptr_->subscribe("image_raw", 1, &CFtldRosNodelet::ImageCallback, this, it_th);

  NODELET_WARN_STREAM("[CFTLD] Image transport in use: " << sub_image_.getTransport());

  if (enable_debug_image_)
  {
    NODELET_INFO("[CFTLD] The debug image is enabled");
    pub_debug_image_ = it_ptr_->advertise("debug_image", 1);
  }

  sub_roi_ = nh_.subscribe("init_roi", 1, &CFtldRosNodelet::InitROICallback, this);
  sub_reset_ = nh_.subscribe("reset", 1, &CFtldRosNodelet::ResetCallback, this);
  pub_track_ = nh_.advertise<cftld_ros::Track>("track", 10);

  do_downsample_ = param_downsample_factor_ < 1.0 && param_downsample_factor_ > 0.0;
  NODELET_INFO("[CFTLD] CFtldRosNodelet is initialized.");
}

}  // namespace cftld_ros

PLUGINLIB_DECLARE_CLASS(cftld_ros, CFtldRosNodelet, cftld_ros::CFtldRosNodelet, nodelet::Nodelet);
