#include <iostream>
#include <cmath>
#include <vector>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <rtt_ros/rtt_ros.h>
#include <rtt_roscomm/rostopic.h>

#include "obstacle_avoidance_potential_field/potentialfield.h"

using namespace std;
using namespace RTT;

class PotentialFielPlanning : public RTT::TaskContext
{

public:
  // In- Out Ports
  OutputPort<geometry_msgs::Point> out_path;
  InputPort<geometry_msgs::Point> in_start_pos, in_obst_pos, in_goal_pos;
  InputPort<std_msgs::Float64> in_obst_size;

  // Datatypes for In- Out Ports
  geometry_msgs::Point path_msg, start_pos_msg, obst_pos_msg, goal_pos_msg;
  std_msgs::Float64 obst_size_msg;

  float delta, K_a, K_r, d0, radius;
  float q[2], o[2], g[2];
  bool started = false;

  PotentialField *p;

  PotentialFielPlanning(const std::string &name) : TaskContext(name),
                                                   out_path("out_path"),
                                                   in_obst_pos("in_obst_pos"),
                                                   in_goal_pos("in_goal_pos"),
                                                   in_start_pos("in_start_pos"),
                                                   in_obst_size("in_obst_size")
  {

    delta = 0.01;
    K_a = 0.5;
    K_r = 5.0;
    d0 = 0.5;
    p = new PotentialField(delta, K_a, K_r, d0);

    this->ports()->addPort(out_path).doc("cordinates of the path");
    out_path.createStream(rtt_roscomm::topic("path_topic"));

    this->ports()->addPort(in_obst_pos).doc("cordinates of obstacle");
    in_obst_pos.createStream(rtt_roscomm::topic("obst_pos_topic"));

    this->ports()->addPort(in_goal_pos).doc("coordinates goal");
    in_goal_pos.createStream(rtt_roscomm::topic("goal_pos_topic"));

    this->ports()->addPort(in_start_pos).doc("coordinates start");
    in_start_pos.createStream(rtt_roscomm::topic("start_pos_topic"));

    this->ports()->addPort(in_obst_size).doc("size of obstacle");
    in_obst_size.createStream(rtt_roscomm::topic("obst_size_topic"));
  }

  void updateHook()
  {

    // read from input ports
    if (in_start_pos.read(start_pos_msg) == RTT::NewData && !started)
    {
      q[0] = start_pos_msg.x;
      q[1] = start_pos_msg.y;
      started = true;
    }
    if (in_obst_pos.read(obst_pos_msg) == RTT::NewData)
    {
      o[0] = obst_pos_msg.x;
      o[1] = obst_pos_msg.y;
    }
    if (in_goal_pos.read(goal_pos_msg) == RTT::NewData)
    {
      g[0] = goal_pos_msg.x;
      g[1] = goal_pos_msg.y;
    }
    if (in_obst_size.read(obst_size_msg) == RTT::NewData)
    {
      radius = obst_size_msg.data / 2.0;
    }

    p->calcPath(q, o, g, radius);
    path_msg.x = q[0];
    path_msg.y = q[1];
    path_msg.z = 0;
    out_path.write(path_msg);
  }
};
ORO_CREATE_COMPONENT(PotentialFielPlanning)
