#include <iostream>
#include <cmath>
#include <vector>
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

float delta = 0.1;
float K_a = 50.0;
float K_r = 100.0;
float d0 = 5.0;
float radius = 1.0;

// def draw(path, obst, r):
//     circle = plt.Circle((obst[0], obst[1]), r)
//     plt.gcf().gca().add_artist(circle)

//     for p in path:
//         plt.scatter(p[0], p[1], s=0.1, c='blue')

//     plt.ylim(0, 50)
//     plt.xlim(0, 50)
//     plt.show()

void draw(vector<vector<float>> path, float *o)
{
    for (int i = 0; i < path.size(); i++)
    {
        vector<float> x = {path[i][0]};
        vector<float> y = {path[i][1]};

        plt::scatter(x, y);
        //cout << path[i][0] << ", " << path[i][1] << endl;
    }
    vector<float> x = {o[0]};
    vector<float> y = {o[1]};
    plt::scatter(x, y, 100);
    plt::show();
}

float magnitude(float *V)
{
    return sqrt(V[0] * V[0] + V[1] * V[1]);
}

float dist(float *p1, float *p2)
{
    return sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]));
}

void F_att(float *q, float *q_g, float K, float *F)
{
    F[0] = -K * (q[0] - q_g[0]);
    F[1] = -K * (q[1] - q_g[1]);
}

void F_rep(float *q, float *q_o, float K, float d0, float radius, float *F)
{
    float d = dist(q, q_o) - radius;
    if (d < d0)
    {

        F[0] = K * (1.0 / d - 1.0 / d0) * (1.0 / (d * d)) * ((q[0] - q_o[0]) / d);
        F[1] = K * (1.0 / d - 1.0 / d0) * (1.0 / (d * d)) * ((q[1] - q_o[1]) / d);
    }
    else
    {
        F[0] = 0;
        F[1] = 0;
    }
}

int main()
{

    float q[] = {10.0, 8.0};
    float q_o[] = {20.0, 15.0};
    float q_g[] = {30.0, 20.0};

    vector<vector<float>> path;
    vector<float> point;
    float d;
    float f_att[2];
    float f_rep[2];
    float f_ges[2];
    float f_magnitude;

    point = {q[0], q[1]};
    path.push_back(point);
    d = dist(q, q_g);

    while (d > 0.5)
    {

        F_att(q, q_g, K_a, f_att);
        F_rep(q, q_o, K_r, d0, radius, f_rep);
        f_ges[0] = f_att[0] + f_rep[0];
        f_ges[1] = f_att[1] + f_rep[1];

        f_magnitude = magnitude(f_ges);

        q[0] += delta * (f_ges[0] / f_magnitude);
        q[1] += delta * (f_ges[1] / f_magnitude);

        point = {q[0], q[1]};
        path.push_back(point);

        d = dist(q, q_g);
        cout << d << endl;
    }

    draw(path, q_o);

    // for (int i = 0; i < path.size(); i++)
    // {
    //     cout << path[i][0] << ", " << path[i][1] << endl;
    // }
    return 0;
}