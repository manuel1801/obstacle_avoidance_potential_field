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

using namespace std;
using namespace RTT;

class PotentialFielPlanning : public RTT::TaskContext
{

public:
  OutputPort<std_msgs::Float64> output_path_x;
  OutputPort<std_msgs::Float64> output_path_y;

  vector<float> path_x, path_y;
  float delta, K_a, K_r, d0, radius;
  float f_x, f_y, d_o, d_g, f_abs, q_x, q_y, g_x, g_y;
  float o_x[2], o_y[2];

  PotentialFielPlanning(const std::string &name) : TaskContext(name),
                                                   output_path_x("path_x"),
                                                   output_path_y("path_x")
  {
    delta = 0.1;
    K_a = 5.0;
    K_r = 100.0;
    d0 = 5.0;
    radius = 1.3;

    this->addPort(output_path_x).doc("x");
    this->addPort(output_path_y).doc("y");

    // Robot Position
    q_x = 10.0;
    q_y = 8.0;

    // Goal Position
    g_x = 45.0;
    g_y = 35.0;

    // Obstacles Position
    o_x[0] = 20.0;
    o_y[0] = 15.0;
    o_x[1] = 30.0;
    o_y[1] = 25.0;

    // Trajectory Path
    path_x.push_back(q_x);
    path_y.push_back(q_y);

    // Distance to Goal
    d_g = sqrt((g_x - q_x) * (g_x - q_x) + (g_y - q_y) * (g_y - q_y));

    log(Info) << "Configuration done," << endlog();
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
    //std_msgs::Float64 fdata;
    if (d_g > 0.5)
    {

      // Calculate Attractive Force of Goal
      f_x = calc_F_att(q_x, g_x, K_a);
      f_y = calc_F_att(q_y, g_y, K_a);

      // Calculate Repulsive Forces of all Obstacles
      for (int i = 0; i < sizeof(o_x) / sizeof(o_x[0]); i++)
      {
        d_o = sqrt((o_x[i] - q_x) * (o_x[i] - q_x) + (o_y[i] - q_y) * (o_y[i] - q_y));
        f_x += calc_F_rep(q_x, o_x[i], K_r, d_o, d0, radius);
        f_y += calc_F_rep(q_y, o_y[i], K_r, d_o, d0, radius);
      }

      // Calculate Magnitude of resulting Force Vector
      f_abs = sqrt(f_x * f_x + f_y * f_y);

      // Increment Robot Position by the gradients
      q_x += delta * (f_x / f_abs);
      q_y += delta * (f_y / f_abs);

      // Add new waypoints to trajectory path
      path_x.push_back(q_x);
      path_y.push_back(q_y);

      // Calculate distance to goal with new position
      d_g = sqrt((g_x - q_x) * (g_x - q_x) + (g_y - q_y) * (g_y - q_y));
      cout << "Distance: " << d_g << ", Position: x:" << q_x << ", x:" << q_y << endl;
    }
  }
};
ORO_CREATE_COMPONENT(PotentialFielPlanning)