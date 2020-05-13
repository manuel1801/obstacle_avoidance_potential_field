#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/menu_handler.h>

#include <tf/tf.h>
#include <math.h>

using namespace std;
using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

bool menu_clicked = false;
geometry_msgs::Point obstacle_pos, start_pos, goal_pos;
Marker path_marker;
std_msgs::Float64 size;
float size_f = 1.0;

vector<float> path_x, path_y, path_z;

Marker makeObstacle(InteractiveMarker &msg)
{

  Marker obstacle;
  if (msg.name == "obstacle")
  {
    obstacle.type = Marker::SPHERE;
  }
  else
  {
    obstacle.type = Marker::CUBE;
  }

  obstacle.scale.x = msg.scale * 0.45;
  obstacle.scale.y = msg.scale * 0.45;
  obstacle.scale.z = msg.scale * 0.45;
  obstacle.color.r = 0.5;
  obstacle.color.g = 0.5;
  obstacle.color.b = 0.5;
  obstacle.color.a = 1.0;

  return obstacle;
}

Marker makeLine(vector<float> path_x, vector<float> path_y)
{
  // auch in funktion wie bei obstacle
  Marker line_strip;
  line_strip.header.frame_id = "/base_frame";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "obstacle_path_planning";
  line_strip.id = 0;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.05;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  for (int i = 0; i < path_x.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = path_x[i];
    p.y = path_y[i];
    p.z = 0;
    line_strip.points.push_back(p);
  }
  return line_strip;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

  // gets executed when object was moved or size was changed

  //check if moved
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    if (feedback->marker_name == "obstacle")
    {
      obstacle_pos.x = feedback->pose.position.x;
      obstacle_pos.y = feedback->pose.position.y;
      obstacle_pos.z = feedback->pose.position.z;
    }
    else if (feedback->marker_name == "start")
    {
      start_pos.x = feedback->pose.position.x;
      start_pos.y = feedback->pose.position.y;
      start_pos.z = feedback->pose.position.z;
    }
    else if (feedback->marker_name == "goal")
    {
      goal_pos.x = feedback->pose.position.x;
      goal_pos.y = feedback->pose.position.y;
      goal_pos.z = feedback->pose.position.z;
    }
  }
  // check if menu
  else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    // feedback contains information which menu entry was clicked
    menu_clicked = true;
    switch (feedback->menu_entry_id)
    {
    case 1:
      size.data = 0.5;
      break;
    case 2:
      size.data = 1.0;
      break;
    case 3:
      size.data = 1.5;
      break;
    case 4:
      size.data = 2.0;
      break;
    case 5:
      size.data = 2.5;
      break;
    default:
      break;
    }
  }
  server->applyChanges();
}

//void makeObstacleMarker(const tf::Vector3 &position, float size, const char *name)
void makeObstacleMarker(const geometry_msgs::Point &position, float size, const char *name)

{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_frame";
  tf::pointTFToMsg(tf::Vector3(position.x, position.y, position.z),
                   int_marker.pose.position);

  int_marker.scale = size;
  int_marker.name = name;
  int_marker.description = name;

  InteractiveMarkerControl control;

  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  control.markers.push_back(makeObstacle(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (name == "obstacle")
    menu_handler.apply(*server, int_marker.name);
}

void pathCB(const geometry_msgs::Point &pathMsg)
{
  path_x.push_back(pathMsg.x);
  path_y.push_back(pathMsg.y);
  path_z.push_back(pathMsg.z);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_visualization_rviz");
  ros::NodeHandle n;
  ros::Rate rate(10);

  // Publish start, goal, obst, position and size to Orocos
  ros::Publisher start_pos_pub = n.advertise<geometry_msgs::Point>("start_pos_topic", 1);
  ros::Publisher goal_pos_pub = n.advertise<geometry_msgs::Point>("goal_pos_topic", 1);
  ros::Publisher obst_pos_pub = n.advertise<geometry_msgs::Point>("obst_pos_topic", 1);
  ros::Publisher obst_size_pub = n.advertise<std_msgs::Float64>("obst_size_topic", 1);

  // Subscrie to Topic for path coordinates sent by Orocos
  ros::Subscriber path_sub = n.subscribe("path_topic", 1000, pathCB);

  // Publish Path coordinates to Rviz
  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("path_marker", 1);

  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("Size: 0.5", &processFeedback);
  menu_handler.insert("Size: 1.0", &processFeedback);
  menu_handler.insert("Size: 1.5", &processFeedback);
  menu_handler.insert("Size: 2.0", &processFeedback);
  menu_handler.insert("Size: 2.5", &processFeedback);

  obstacle_pos.x = 0.0;
  obstacle_pos.y = 0.0;
  obstacle_pos.z = 0.0;

  start_pos.x = -4.0;
  start_pos.y = -4.0;
  start_pos.z = 0.0;

  goal_pos.x = 4.0;
  goal_pos.y = 4.0;
  goal_pos.z = 0.0;

  size.data = 1.0;

  makeObstacleMarker(obstacle_pos, size.data, "obstacle");
  makeObstacleMarker(start_pos, size.data, "start");
  makeObstacleMarker(goal_pos, size.data, "goal");

  server->applyChanges();

  while (ros::ok())
  {

    // publish obst,start,goal pos to orocos
    start_pos_pub.publish(start_pos);
    goal_pos_pub.publish(goal_pos);
    obst_pos_pub.publish(obstacle_pos);
    obst_size_pub.publish(size);

    // recreate obstacle with new size
    if (menu_clicked)
    {
      makeObstacleMarker(obstacle_pos, size.data, "obstacle");
      server->applyChanges();
      menu_clicked = false;
    }

    // publish path as line to rviz
    path_marker = makeLine(path_x, path_y);
    obstacle_pub.publish(path_marker);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}