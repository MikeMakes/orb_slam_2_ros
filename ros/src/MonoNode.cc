#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    //OdomHandler odomhandler = OdomHandler();
    //ros::Subscriber odom_subscriber = node_handle.subscribe("/bebop/odom", 10, &OdomHandler::OdomCallback, &odomhandler); //odom for scale init

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}
/*
OdomHandler::OdomHandler(){
  ROS_WARN ("OdomHandler::OdomHandler");
  _dist=0;
  _px=0;
  _py=0;
  _pz=0;
}*/

MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  odom_subscriber = node_handle.subscribe("/bebop/odom", 10, &MonoNode::OdomCallback, this); //odom for scale init
  camera_info_topic_ = "/camera/camera_info";
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec(), _dist);

  Update ();
}

void MonoNode::OdomCallback (const nav_msgs::Odometry::ConstPtr& msg) {
  double px = msg->pose.pose.position.x;
  double py = msg->pose.pose.position.y;
  double pz = msg->pose.pose.position.z;
  //_dist = sqrt(pow(px-_px,2)+pow(py-_py,2)+pow(pz-_pz,2));
  _dist = sqrt(pow(px,2)+pow(py,2)+pow(pz,2));
  _px=px;
  _py=py;
  _pz=pz;
  ROS_WARN ("Dist: %f", (float)_dist);
}
