#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/Marker.h>
#include <iostream>

using namespace std;
float p_x, p_y, o_x, o_y, o2_x, o2_y;
float scale_factor = 0.1;

vector<float> path_x, path_y;

void pathXCallback(const std_msgs::Float64 &pathXMsg)
{
  p_x = pathXMsg.data * scale_factor;
  path_x.push_back(p_x);
  //ROS_INFO("path msg received: x=%f", pathXMsg.data);
}
void pathYCallback(const std_msgs::Float64 &pathYMsg)
{
  p_y = pathYMsg.data * scale_factor;
  path_y.push_back(p_y);
  //ROS_INFO("path msg received: y=%f", pathYMsg.data);
}
void obstXCallback(const std_msgs::Float64 &obstXMsg)
{
  o_x = obstXMsg.data * scale_factor;
  //ROS_INFO("obstacle msg received: x=%f", obstXMsg.data);
}
void obstYCallback(const std_msgs::Float64 &obstYMsg)
{
  o_y = obstYMsg.data * scale_factor;
  //ROS_INFO("obstacle msg received: y=%f", obstYMsg.data);
}
void obst2XCallback(const std_msgs::Float64 &obst2XMsg)
{
  o2_x = obst2XMsg.data * scale_factor;
  //ROS_INFO("obstacle msg received: x=%f", obstXMsg.data);
}
void obst2YCallback(const std_msgs::Float64 &obst2YMsg)
{
  o2_y = obst2YMsg.data * scale_factor;
  //ROS_INFO("obstacle msg received: y=%f", obstYMsg.data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::Rate rate(10);

  ros::Subscriber path_sub_x = n.subscribe("path_topic_x", 1000, pathXCallback);
  ros::Subscriber path_sub_y = n.subscribe("path_topic_y", 1000, pathYCallback);
  ros::Subscriber obstacle_sub_x = n.subscribe("obstacle_topic_x", 1000, obstXCallback);
  ros::Subscriber obstacle_sub_y = n.subscribe("obstacle_topic_y", 1000, obstYCallback);
  ros::Subscriber obstacle2_sub_x = n.subscribe("obstacle2_topic_x", 1000, obst2XCallback);
  ros::Subscriber obstacle2_sub_y = n.subscribe("obstacle2_topic_y", 1000, obst2YCallback);

  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("obstacle_marker", 1);

  p_x = p_y = o_x = o_y = o2_x = o2_y = 0.0;

  while (ros::ok())
  {
    //publish to rviz
    visualization_msgs::Marker obstacle, obstacle2, line_strip;
    obstacle.header.frame_id = obstacle2.header.frame_id = line_strip.header.frame_id = "/obstacle_frame";
    obstacle.header.stamp = obstacle2.header.stamp = line_strip.header.stamp = ros::Time::now();
    obstacle.ns = obstacle2.ns = line_strip.ns = "obstacle_path_planning";
    obstacle.id = 0;
    obstacle2.id = 1;
    line_strip.id = 2;

    obstacle.action = obstacle2.action = line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    obstacle.type = visualization_msgs::Marker::SPHERE;
    obstacle2.type = visualization_msgs::Marker::SPHERE;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.08;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    obstacle.pose.position.x = o_x;
    obstacle.pose.position.y = o_y;
    obstacle.pose.position.z = 0;
    obstacle.pose.orientation.x = 0.0;
    obstacle.pose.orientation.y = 0.0;
    obstacle.pose.orientation.z = 0.0;
    obstacle.pose.orientation.w = 1.0;
    obstacle.scale.x = 0.3;
    obstacle.scale.y = 0.3;
    obstacle.scale.z = 0.3;
    obstacle.color.r = 0.0f;
    obstacle.color.g = 1.0f;
    obstacle.color.b = 0.0f;
    obstacle.color.a = 1.0;

    obstacle2.pose.position.x = o2_x;
    obstacle2.pose.position.y = o2_y;
    obstacle2.pose.position.z = 0;
    obstacle2.pose.orientation.x = 0.0;
    obstacle2.pose.orientation.y = 0.0;
    obstacle2.pose.orientation.z = 0.0;
    obstacle2.pose.orientation.w = 1.0;
    obstacle2.scale.x = 0.3;
    obstacle2.scale.y = 0.3;
    obstacle2.scale.z = 0.3;
    obstacle2.color.r = 0.0f;
    obstacle2.color.g = 1.0f;
    obstacle2.color.b = 0.0f;
    obstacle2.color.a = 1.0;

    obstacle.lifetime = ros::Duration();
    obstacle2.lifetime = ros::Duration();

    // while (path_sub_x.getNumPublishers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("waiting for orocos to send first data");
    //   sleep(1);
    // }

    // while (obstacle_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }

    for (int i = 0; i < path_x.size(); ++i)
    {

      geometry_msgs::Point p;
      p.x = path_x[i];
      p.y = path_y[i];
      p.z = 0;

      line_strip.points.push_back(p);
    }
    obstacle_pub.publish(obstacle);
    obstacle_pub.publish(obstacle2);
    obstacle_pub.publish(line_strip);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}