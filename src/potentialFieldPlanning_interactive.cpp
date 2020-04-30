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

using namespace std;
using namespace RTT;

class PotentialFielPlanningInt : public RTT::TaskContext
{

public:
  // In- Out Ports
  OutputPort<geometry_msgs::Point> out_path;

  InputPort<geometry_msgs::Point> in_start_pos, in_obst_pos, in_goal_pos;
  InputPort<std_msgs::Float64> in_obst_size;

  // Datatypes for In- Out Ports
  geometry_msgs::Point path_msg, start_pos_msg, obst_pos_msg, goal_pos_msg;
  std_msgs::Float64 obst_size_msg;

  float delta, K_a, K_r, d0;
  float f_x, f_y, d_o, d_g, f_abs, q_x, q_y, g_x, g_y;
  float o_x[1], o_y[1], radius[1];
  int N = 1;
  bool started = false;

  PotentialFielPlanningInt(const std::string &name) : TaskContext(name),
                                                      out_path("out_path"),
                                                      in_obst_pos("in_obst_pos"),
                                                      in_goal_pos("in_goal_pos"),
                                                      in_start_pos("in_start_pos"),
                                                      in_obst_size("in_obst_size")

  {

    delta = 0.01;
    K_a = 0.5;
    K_r = 10.0;
    d0 = 0.5;

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

  float calc_F_att(float q, float q_g, float K)
  {
    return -K * (q - q_g);
  }

  float calc_F_rep(float q, float q_o, float K, float d, float d0, float radius)
  {
    d -= radius;
    if (d < d0)
    {
      return K * (1.0 / d - 1.0 / d0) * (1.0 / (d * d)) * ((q - q_o) / d);
    }
    else
    {
      return 0;
    }
  }

  void updateHook()
  {

    // read from input ports
    if (in_start_pos.read(start_pos_msg) == RTT::NewData && !started)
    {
      q_x = start_pos_msg.x;
      q_y = start_pos_msg.y;
      started = true;
    }

    if (in_obst_pos.read(obst_pos_msg) == RTT::NewData)
    {
      o_x[0] = obst_pos_msg.x;
      o_y[0] = obst_pos_msg.y;
      //log(Info) << "receied: " << obst_pos_msg.x << endlog();
    }
    if (in_goal_pos.read(goal_pos_msg) == RTT::NewData)
    {
      g_x = goal_pos_msg.x;
      g_y = goal_pos_msg.y;
      //log(Info) << "receied: " << goal_pos_msg.x << endlog();
    }
    if (in_obst_size.read(obst_size_msg) == RTT::NewData)
    {
      radius[0] = obst_size_msg.data / 2.0;
      //log(Info) << "receied: " << obst_size_msg.data << endlog();
    }

    d_g = sqrt((g_x - q_x) * (g_x - q_x) + (g_y - q_y) * (g_y - q_y));

    if (d_g > 0.1)
    {
      path_msg.x = q_x;
      path_msg.y = q_y;
      path_msg.z = 0;
      out_path.write(path_msg);

      // Calculate Attractive Force of Goal
      f_x = calc_F_att(q_x, g_x, K_a);
      f_y = calc_F_att(q_y, g_y, K_a);

      // Calculate Repulsive Forces of all Obstacles
      for (int i = 0; i < N; i++)
      {
        d_o = sqrt((o_x[i] - q_x) * (o_x[i] - q_x) + (o_y[i] - q_y) * (o_y[i] - q_y));
        f_x += calc_F_rep(q_x, o_x[i], K_r, d_o, d0, radius[i]);
        f_y += calc_F_rep(q_y, o_y[i], K_r, d_o, d0, radius[i]);
      }

      // Calculate Magnitude of resulting Force Vector
      f_abs = sqrt(f_x * f_x + f_y * f_y);

      // Increment Robot Position by the gradients
      q_x += delta * (f_x / f_abs);
      q_y += delta * (f_y / f_abs);

      // Add new waypoints to trajectory path
      //path_x.push_back(q_x);
      //path_y.push_back(q_y);
    }
  }
};
ORO_CREATE_COMPONENT(PotentialFielPlanningInt)
