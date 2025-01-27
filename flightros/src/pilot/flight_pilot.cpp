#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::BASICLINE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    send_frame_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.4, 0.0, -0.3);
  // Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  Matrix<3, 3> R_BC = (Matrix<3, 3>() << 0.0, 0.707, 0.707,
                                        -1.0, 0.0, 0.0,
                                         0.0, -0.707, 0.707).finished();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, true, false});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);

  // add third person view rgb camera
  third_person_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC2(-5, 0.0, 2);
  Matrix<3, 3> R_BC2 = (Matrix<3, 3>() << 0.0, 1.0, 0.0,
                                        -1.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0).finished();
  std::cout << R_BC << std::endl;
  third_person_camera_->setFOV(90);
  third_person_camera_->setWidth(720);
  third_person_camera_->setHeight(480);
  third_person_camera_->setRelPose(B_r_BC2, R_BC2);
  third_person_camera_->setPostProcesscing(
    std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(third_person_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // initialize publishers (using public nodehandle to be in correct ns)
  image_transport::ImageTransport it(nh_);
  rgb_pub_ = it.advertise("rgb", 1);
  depth_pub_ = it.advertise("depth", 1);
  segmentation_pub_ = it.advertise("segmentation", 1);
  opticalflow_pub_ = it.advertise("opticalflow", 1);
  third_person_pub_ = it.advertise("rgb_third_person", 1);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  uint16_t receive_frame_id;
  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(send_frame_id_);
    receive_frame_id = unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      ROS_INFO("COLLISION");
    }
  }

  ros::Time timestamp;
  if (send_frame_id_ == receive_frame_id) 
  {
    // it means the received image is corresponding to the current send pose, 
    // otherwise it was the image from last update
    timestamp = msg->header.stamp;
  }
  else
  {
    timestamp = timestamp_prev_;
  }
  send_frame_id_ += 1; // TODO: inside if or here??
  timestamp_prev_ = msg->header.stamp;

  cv::Mat img;
  rgb_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = timestamp;
  rgb_pub_.publish(rgb_msg);

  rgb_camera_->getDepthMap(img);
  sensor_msgs::ImagePtr depth_msg =
    cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
  depth_msg->header.stamp = timestamp;
  depth_pub_.publish(depth_msg);

  rgb_camera_->getSegmentation(img);
  sensor_msgs::ImagePtr segmentation_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  segmentation_msg->header.stamp = timestamp;
  segmentation_pub_.publish(segmentation_msg);

  third_person_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr third_person_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  third_person_msg->header.stamp = timestamp;
  third_person_pub_.publish(third_person_msg);

}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  

}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros