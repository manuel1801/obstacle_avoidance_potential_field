from matplotlib import pyplot as plt
import math

delta = 0.1
K_a = 50.0
K_r = 100.0
d0 = 5.0
radius = 1.0


def draw(path, obst, r):
    for o in obst:
        circle = plt.Circle((o[0], o[1]), r)
        plt.gcf().gca().add_artist(circle)

    for p in path:
        plt.scatter(p[0], p[1], s=0.1, c='blue')

    plt.ylim(0, 50)
    plt.xlim(0, 50)
    plt.show()


def magnitude(V):
    return math.sqrt(V[0]**2 + V[1]**2)


def dist(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def F_att(q, q_g, K):
    f_att_x = - K * (q[0] - q_g[0])
    f_att_y = - K * (q[1] - q_g[1])
    return f_att_x, f_att_y


def F_rep(q, q_o, K, d0, radius):
    d = dist(q, q_o) - radius
    if d < d0:
        f_rep_x = K * (1.0/d - 1.0/d0) * (1.0/(d**2)) * ((q[0] - q_o[0])/d)
        f_rep_y = K * (1.0/d - 1.0/d0) * (1.0/(d**2)) * ((q[1] - q_o[1])/d)
    else:
        f_rep_x = 0
        f_rep_y = 0
    return f_rep_x, f_rep_y


q = [10, 8]
q_o = [[20, 15], [30, 25]]
q_g = [45, 35]


path = []

path.append([q[0], q[1]])
d = dist(q, q_g)

while d > 0.5:
    f_att = F_att(q, q_g, K_a)
    f_rep = []
    f_ges_x = 0
    f_ges_y = 0

    for o in q_o:
        f_rep.append(F_rep(q, o, K_r, d0, radius))
        f_ges_x


    f_ges = [f_att[0] + f_rep[0] + f_rep2[0], f_att[1] + f_rep[1] + f_rep2[1]]
    f_magnitude = magnitude(f_ges)

    q[0] += delta * (f_ges[0] / f_magnitude)
    q[1] += delta * (f_ges[1] / f_magnitude)

    path.append([q[0], q[1]])
    d = dist(q, q_g)

o_all = [q_o, q_o2]
draw(path, o_all, radius)
