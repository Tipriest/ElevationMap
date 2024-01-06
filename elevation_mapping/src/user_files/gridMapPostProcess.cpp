#include "ros/ros.h"
#include <iostream>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>
#include <string>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

using namespace std;



float obstatleMin, obstatleMax, mapResolution; //用来滤除的障碍物的下界和上界，在这个高度范围的会被认为是障碍物，其他地方会被认为是空地

grid_map::GridMap map_({"elevation"});
ros::Publisher gridMapPub;
void process(grid_map::GridMap &map_p);
void publish();

//gridMapCallback回调函数
void gridMapCallback(const grid_map_msgs::GridMap& msgs)
{
    // cout<<msgs.info.length_x<<' '<<msgs.info.length_y<<' '<<msgs.info.resolution<<endl;
    // cout<<msgs.info.pose.position.x<<' '<<msgs.info.pose.position.y<<endl;
    
    grid_map::GridMapRosConverter::fromMessage(msgs, map_,{"elevation"},false,false);
    //map_.setStartIndex({0,0});
    //map_.setPosition({0,0});
    process(map_);
    publish();

}

void publish()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridMapPub.publish(message);
  //ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

void process(grid_map::GridMap &map_p)
{
    //grid map迭代器
    grid_map::GridMapIterator iterator(map_p);
    //遍历grid map
    for (iterator; !iterator.isPastEnd(); ++iterator)
    {
        //获取当前cell的索引
        grid_map::Index index = *iterator;
        //获取当前cell的值
        float value = map_p.at("elevation", index);
        //判断当前cell是否为nan
        if (isnan(value))
        {
            //如果是nan，将当前cell的值设为0
            map_p.at("elevation", index) = 0;
        }else if(value>obstatleMin){
            map_p.at("elevation", index) = 2.6;
            std::cout<<"obstacle_min: "<<obstatleMin<<std::endl;
        }else{
            map_p.at("elevation", index) = 0;
            
        }
    }
}

//ros主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridMapPostProcess");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);

      /*  Fsm param  */
    float mapSizeX = nh.param<float>("mapSizeX", 30.0);
    float mapSizeY = nh.param<float>("mapSizeY", 30.0);
    mapResolution = nh.param<float>("mapResolution", 0.2);
    obstatleMin = nh.param<float>("obstatleMin", 0.05);
    obstatleMax = nh.param<float>("obstatleMax", 0.30);
    std::string mapInTopicName = nh.param<std::string>("mapInTopicName", "/elevation_mapping/elevation_map");
    std::string mapOutTopicName = nh.param<std::string>("mapOutTopicName", "/elevation_mapping/elevation_map_post_process");
    std::string mapFrameId = nh.param<std::string>("mapFrameId", "map");
    std::string robotFrameId = nh.param<std::string>("robotFrameId", "base_link");

    //定义gridmap
    map_.setFrameId(robotFrameId);
    map_.setGeometry(grid_map::Length(mapSizeX, mapSizeY), mapResolution);
    map_.setPosition(grid_map::Position(0, 0));
    map_.setTimestamp(ros::Time::now().toNSec());
    ros::Subscriber gridMapSub = nh.subscribe(mapInTopicName, 1, &gridMapCallback);
    gridMapPub = nh.advertise<grid_map_msgs::GridMap>(mapOutTopicName, 1);

    ros::spin();
    loop_rate.sleep();
    return 0;
}