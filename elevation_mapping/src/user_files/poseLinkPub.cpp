#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

ros::Time last_timestamp;  // 上一个时间戳
void odomCallback(const nav_msgs::Odometry& msg);

int main(int argc, char** argv){
    ros::init(argc, argv, "poseLinkPub");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("torso_odom", 10, odomCallback);
    ros::spin();
}
/**
 * @brief 接收机器人gazebo插件的torso Odometry的消息，回调函数中发布tf坐标关系，父坐标系为world，子坐标系为base_link
 * @param[in] msg           My Param doc
 * @author Tipriest (a1503741059@163.com)
 */
void odomCallback(const nav_msgs::Odometry& msg) {

    if (msg.header.stamp == last_timestamp)
        {
            ROS_WARN("Duplicate timestamp detected. Ignoring the data.");
            return;
        }

    // 更新上一个时间戳
    last_timestamp = msg.header.stamp;
    //std::cout<<"Timestamp: " << msg.header.stamp<<std::endl;
    static tf::TransformBroadcaster br;
    // 提取Odometry消息中的位置和姿态信息
    const geometry_msgs::Pose& pose = msg.pose.pose;
    const geometry_msgs::Point& pos = pose.position;
    const geometry_msgs::Quaternion& ori = pose.orientation;

    // 创建TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = pos.x;
    transformStamped.transform.translation.y = pos.y;
    transformStamped.transform.translation.z = pos.z;
    transformStamped.transform.rotation.x = ori.x;
    transformStamped.transform.rotation.y = ori.y;
    transformStamped.transform.rotation.z = ori.z;
    transformStamped.transform.rotation.w = ori.w;
    
    //send the transform
    br.sendTransform(transformStamped);
}