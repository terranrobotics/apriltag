#include "apriltag_ros/apriltag_detector_node.h"

#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <boost/thread/lock_guard.hpp>
#include <boost/asio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

namespace apriltag_ros {

using namespace sensor_msgs;
using apriltag_msgs::ApriltagArrayStamped;

thread_local boost::shared_ptr<ApriltagDetector> ApriltagDetectorNode::detector_ = nullptr;

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle &pnh)
    : pnh_(pnh), it_(pnh_), cfg_server_(pnh) {
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));

  // Setup connect callback
  auto connect_cb = boost::bind(&ApriltagDetectorNode::ConnectCb, this);
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_tags_ =
      pnh_.advertise<ApriltagArrayStamped>("tags", 1, connect_cb, connect_cb);
  pub_disp_ = it_.advertise("disp", 1, connect_cb, connect_cb);

  // // Initialize thread pool with 4 threads
  // thread_pool_ = std::make_unique<boost::asio::thread_pool>(4);
}

void ApriltagDetectorNode::ImageCb(const ImageConstPtr &image_msg) {
  boost::asio::post(*thread_pool_, [this, image_msg]() {
    // Lazy initialization: if the thread-local detector is not yet created, create one
    if (!detector_) {
        // detector_ = std::make_unique<ApriltagDetector>(ApriltagDetector::Create(current_detector_type_, current_detector_family_));
        detector_ = ApriltagDetector::Create(current_detector_type_, current_detector_family_);
        detector_->set_black_border(black_border_);
        detector_->set_decimate(decimate_);
        detector_->set_nthreads(nthreads_);
    }

    const auto gray =
        cv_bridge::toCvShare(image_msg, image_encodings::MONO8)->image;

    // Detect
    auto apriltags = detector_->Detect(gray);

    // Publish apriltags
    auto apriltag_array_msg = boost::make_shared<ApriltagArrayStamped>();
    apriltag_array_msg->header = image_msg->header;
    apriltag_array_msg->apriltags = apriltags;
    pub_tags_.publish(apriltag_array_msg);

    // Publish detection image if needed
    if (pub_disp_.getNumSubscribers()) {
      cv::Mat disp;
      cv::cvtColor(gray, disp, CV_GRAY2BGR);
      DrawApriltags(disp, apriltags);
      cv_bridge::CvImage cv_img(image_msg->header, image_encodings::BGR8, disp);
      pub_disp_.publish(cv_img.toImageMsg());
    }
  });
}

void ApriltagDetectorNode::ConfigCb(ConfigT &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  if (level < 0 || config_.type != config.type ||
      config_.family != config.family) {
    // TODO: This maybe unsafe if someone changes the order in cfg
    current_detector_type_ = static_cast<DetectorType>(config.type);
    current_detector_family_ = static_cast<TagFamily>(config.family);
  }
  black_border_ = config.black_border;
  decimate_ = config.decimate;
  nthreads_ = config.nthreads;

  int worker_count = config.worker_count;
  // initialize the threadpool, but also cleanup previous one if needed
  if (thread_pool_) {
    thread_pool_.reset();
  }
  thread_pool_.reset(new boost::asio::thread_pool(worker_count));

  // show how many workers we have
  ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
           ("Using " + std::to_string(worker_count) + " workers").c_str());

  // Save config
  config_ = config;
}

void ApriltagDetectorNode::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_tags_.getNumSubscribers() == 0 &&
      pub_disp_.getNumSubscribers() == 0) {
    // Shutting down if nobody subscribes
    ROS_DEBUG("%s: No subscribers, shutting down", pnh_.getNamespace().c_str());
    sub_image_.shutdown();
  } else if (!sub_image_) {
    ROS_DEBUG("%s: Resubscribing", pnh_.getNamespace().c_str());
    sub_image_ =
        it_.subscribe("image", 1, &ApriltagDetectorNode::ImageCb, this);
  }
}

} // namespace apriltag_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");

  try {
    apriltag_ros::ApriltagDetectorNode node(pnh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
