#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// 发布消息
ros::Publisher pub;

void tfCallback(const tf::TransformListener& listener);

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::PoseStamped>("/robotPose", 10);

  tf::TransformListener listener(ros::Duration(0.01));  // 设置缓存时间为0.01秒

  ros::Rate rate(50);  // 设置循环频率为50Hz

  while (ros::ok()) {
    tfCallback(listener);

    rate.sleep();
  }

  return 0;
}
void tfCallback(const tf::TransformListener& listener) {
  // 设置源坐标系和目标坐标系
  std::string source_frame = "world";
  std::string target_frame = "base_link";

  tf::StampedTransform transform;
  try {
    listener.waitForTransform(source_frame, target_frame, ros::Time(0), ros::Duration(0.01));
    listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // 提取位置信息
  double x = transform.getOrigin().x();
  double y = transform.getOrigin().y();
  double z = transform.getOrigin().z();

  // 提取姿态信息
  tf::Quaternion q = transform.getRotation();
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 创建并填充geometry_msgs::PoseStamped消息
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = source_frame;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  pub.publish(pose_msg);
  // std::cout<< "has send pose info"<<"x: "<<x<<"y: "<<y<<"z: "<<z<< std::endl;
}
