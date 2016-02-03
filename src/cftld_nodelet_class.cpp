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

namespace cftld_ros
{

CFtldRosNodelet::CFtldRosNodelet()
  : param_cfg_file_(""),
    param_downsample_factor_(1.0),
    do_downsample_(false),
    tracking_state_(TRACKING_STATE_UNINITED),
    tld_ptr_(new tld::TLD()),
    param_show_output_(true),
    param_tld_threshold_(0.5),
    pram_tld_seed_(0),
    track_id_(1)
{
  //
}

void CFtldRosNodelet::UpdateParams()
{
   GetParam<bool>(private_nh_, "enable_debug_image", enable_debug_image_, false);
   GetParam<std::string>(private_nh_, "tld_config_file", param_cfg_file_, std::string(""));
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
  NODELET_INFO_STREAM("init done 1: " << tld_settings_.m_useDsstTracker);
  tld_ptr_->trackerEnabled = tld_settings_.m_trackerEnabled;
  tld_ptr_->detectorEnabled = tld_settings_.m_detectorEnabled;
  tld_ptr_->init(tld_settings_.m_useDsstTracker);
  tld_ptr_->alternating = tld_settings_.m_alternating;
  tld_ptr_->learningEnabled = tld_settings_.m_learningEnabled;

  NODELET_INFO("init done 2");
//  main->showOutput = m_settings.m_showOutput;
//  main->showTrajectory = (m_settings.m_trajectory) ? true : false;
//  main->trajectoryLength = m_settings.m_trajectory;
//  main->printResults = (m_settings.m_printResults.empty()) ? NULL : m_settings.m_printResults.c_str();
//  main->saveDir = (m_settings.m_outputDir.empty()) ? NULL : m_settings.m_outputDir.c_str();
//  main->threshold = m_settings.m_threshold;
//  main->showNotConfident = m_settings.m_showNotConfident;


//  main->selectManually = m_settings.m_selectManually;
//  main->seed = m_settings.m_seed;

  ROS_ASSERT(tld_ptr_->detectorCascade);
  tld::DetectorCascade *detectorCascade = tld_ptr_->detectorCascade;
  detectorCascade->varianceFilter->enabled = tld_settings_.m_varianceFilterEnabled;
  detectorCascade->ensembleClassifier->enabled = tld_settings_.m_ensembleClassifierEnabled;
  detectorCascade->nnClassifier->enabled = tld_settings_.m_nnClassifierEnabled;

  NODELET_INFO("init done 3");
  // classifier
  detectorCascade->useShift = tld_settings_.m_useProportionalShift;
  detectorCascade->shift = tld_settings_.m_proportionalShift;
  detectorCascade->minScale = tld_settings_.m_minScale;
  detectorCascade->maxScale = tld_settings_.m_maxScale;
  detectorCascade->minSize = tld_settings_.m_minSize;
  detectorCascade->numTrees = tld_settings_.m_numTrees;
  detectorCascade->numFeatures = tld_settings_.m_numFeatures;
  detectorCascade->nnClassifier->thetaTP = tld_settings_.m_thetaP;
  detectorCascade->nnClassifier->thetaFP = tld_settings_.m_thetaN;

  NODELET_INFO("init done");
  return true;
}

void CFtldRosNodelet::ResetCallback(const std_msgs::EmptyConstPtr& empty_msg_ptr)
{
  NODELET_WARN("[CFTLD] Reset requested");
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
    ROS_ERROR("[CFTLD] Tracker object is NULL");
    return;
  }

  if (frame_input_.empty())
  {
    ROS_ERROR("[CFTLD] No input frame has been received or input frame is empty!");
    return;
  }

  // Bad API design of TLD
  cv::Rect bb(roi_msg_ptr->x_offset, roi_msg_ptr->y_offset,
              roi_msg_ptr->width, roi_msg_ptr->height);
  tld_ptr_->selectObject(frame_input_, &bb);
  tracking_state_ = TRACKING_STATE_INITED;
}

void CFtldRosNodelet::ImageCallback(const sensor_msgs::ImageConstPtr &img_msg_ptr)
{
  if (!tld_ptr_)
  {
    NODELET_FATAL("[CFTLD] Tracker is NULL");
    return;
  }
  try
  {
    NODELET_INFO("1");
    if (sensor_msgs::image_encodings::isColor(img_msg_ptr->encoding))
    {
      NODELET_INFO("1 Col");
      NODELET_WARN_ONCE("[CFTLD] Input image is BGR8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      NODELET_INFO("1 Gray");
      NODELET_WARN_ONCE("[CFTLD] Input image is MONO8");
      frame_input_cvptr_ = cv_bridge::toCvShare(img_msg_ptr, sensor_msgs::image_encodings::MONO8);
    }

    NODELET_INFO("2");
    frame_input_ = frame_input_cvptr_->image;

    if (do_downsample_)
    {
      NODELET_INFO("#");
      cv::resize(frame_input_, frame_input_, cv::Size(0, 0),
                 param_downsample_factor_, param_downsample_factor_, cv::INTER_CUBIC);
    }

    if (frame_input_.channels() == 1)
    {
      NODELET_INFO("$$$");
      cv::cvtColor(frame_input_, frame_input_, cv::COLOR_GRAY2BGR);
    }

    if (tracking_state_ == TRACKING_STATE_UNINITED)
    {
      NODELET_INFO("[CFTLD] The tracker has not yet been initialized.");
    }
    else
    {
      tld_ptr_->processImage(frame_input_);
      if (tld_ptr_->currBB)
      {
        NODELET_INFO_STREAM("[CFTLD] Tracking: " << *(tld_ptr_->currBB));
      }
      else
      {
        NODELET_WARN("[CFTLD] Null BB");
      }
    }

    // TODO Check the sanity of tld_ptr_->currBB

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
//      cv_bridge::CvImagePtr frame_debug_cvi_(new cv_bridge::CvImage(header,
//                                                                    sensor_msgs::image_encodings::BGR8,
//                                                                    frame_debug_));

      ROS_INFO_STREAM("Output Mat " << frame_debug_.cols << " " << frame_debug_.rows);
      pub_debug_image_.publish(debug_img_msg);
    }
  }
  catch (const cv_bridge::Exception& e)
  {
    NODELET_ERROR_STREAM("[CFTLD] cv_bridge exception: " << e.what());
  }
}

void CFtldRosNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  UpdateParams();

  NODELET_INFO_STREAM("[CFTLD] Initializing CFtld from " << param_cfg_file_);
  if (!InitCFtld(param_cfg_file_))
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

  do_downsample_ = param_downsample_factor_ < 1.0 && param_downsample_factor_ > 0.0;
  NODELET_INFO("[CFTLD] CFtldRosNodelet is initialized.");
}

}  // namespace cftld_ros

PLUGINLIB_DECLARE_CLASS(cftld_ros, CFtldRosNodelet, cftld_ros::CFtldRosNodelet, nodelet::Nodelet);
