#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <geometry_msgs/Point.h>

using namespace std;

float scale_factor = 0.1;
float o_1[3], o_2[3];
vector<float> path_x, path_y, path_z;

void pathCB(const geometry_msgs::Point &pathMsg)
{
  path_x.push_back(pathMsg.x * scale_factor);
  path_y.push_back(pathMsg.y * scale_factor);
  path_z.push_back(pathMsg.z * scale_factor);
}

void obstacle1CB(const geometry_msgs::Point &obst1Msg)
{
  o_1[0] = obst1Msg.x * scale_factor;
  o_1[1] = obst1Msg.y * scale_factor;
  o_1[2] = obst1Msg.z * scale_factor;
  // ROS_INFO("obstacle 1 x=%f, y=%f, z=%f", o_1[0], o_1[1], o_1[2]);
}
void obstacle2CB(const geometry_msgs::Point &obst2Msg)
{
  o_2[0] = obst2Msg.x * scale_factor;
  o_2[1] = obst2Msg.y * scale_factor;
  o_2[2] = obst2Msg.z * scale_factor;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_visualization_rviz");
  ros::NodeHandle n;
  ros::Rate rate(10);

  // Subscrie to Coordinate Topics sent by Orocos
  ros::Subscriber path_sub = n.subscribe("path_topic", 1000, pathCB);
  ros::Subscriber obstacle1_sub = n.subscribe("obstacle1_topic", 1000, obstacle1CB);
  ros::Subscriber obstacle2_sub = n.subscribe("obstacle2_topic", 1000, obstacle2CB);

  // Publish to Rviz
  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("obstacle_marker", 1);

  while (ros::ok())
  {

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

    obstacle.pose.position.x = o_1[0];
    obstacle.pose.position.y = o_1[1];
    obstacle.pose.position.z = o_1[2];
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
    obstacle.lifetime = ros::Duration();
    obstacle_pub.publish(obstacle);

    obstacle2.pose.position.x = o_2[0];
    obstacle2.pose.position.y = o_2[1];
    obstacle2.pose.position.z = o_2[2];
    obstacle2.pose.orientation.x = 0.0;
    obstacle2.pose.orientation.y = 0.0;
    obstacle2.pose.orientation.z = 0.0;
    obstacle2.pose.orientation.w = 1.0;
    obstacle2.scale.x = 0.3;
    obstacle2.scale.y = 0.3;
    obstacle2.scale.z = 0.3;
    obstacle2.color.r = 1.0f;
    obstacle2.color.g = 0.0f;
    obstacle2.color.b = 0.0f;
    obstacle2.color.a = 1.0;
    obstacle2.lifetime = ros::Duration();
    obstacle_pub.publish(obstacle2);

    for (int i = 0; i < path_x.size(); ++i)
    {

      geometry_msgs::Point p;
      p.x = path_x[i];
      p.y = path_y[i];
      p.z = 0;
      line_strip.points.push_back(p);
    }
    obstacle_pub.publish(line_strip);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}