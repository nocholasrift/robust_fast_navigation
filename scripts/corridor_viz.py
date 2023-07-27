#! /usr/bin/env python3

import os
import sys
import math
import rospy
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

from pydrake import geometry as g

class datum:
    def __init__(self, odom, obs, corridor, helper, traj):
        self.odom = odom
        self.obs = obs
        self.corridor = corridor
        self.helper = helper
        self.traj = traj

    def clean_odom(self):
        return np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])

    def clean_obs(self):
        obs = []
        for point in self.obs.points:
            obs.append([point.x, point.y])

        return np.array(obs)


    def clean_corridor(self, is_helper):
        polys = []
        tmp_poly_A = []
        tmp_poly_b = []

        cleaning_list = self.corridor.poses if not is_helper else self.helper.poses
        for pose in cleaning_list:
            plane = pose.orientation

            # separator plane
            if math.fabs(plane.x) < 1e-6 and math.fabs(plane.y) < 1e-6 and math.fabs(plane.z) < 1e-6 and math.fabs(plane.w) < 1e-6:

                A = np.array(tmp_poly_A)
                b = -np.array(tmp_poly_b)
                # for some reason this flips y...
                hPoly = g.optimization.HPolyhedron(A,b)
                vPoly = g.optimization.VPolytope(hPoly).GetMinimalRepresentation()
                polys.append([hPoly, vPoly])

                tmp_poly_A.clear()
                tmp_poly_b.clear()
                continue

            # z plane, don't care about that one
            if math.fabs(plane.x) < 1e-6 and math.fabs(plane.y) < 1e-6:
                continue

            tmp_poly_A.append([plane.x, plane.y])
            tmp_poly_b.append(plane.w)
            
        return polys

    def clean_traj(self):
        
        points = []
        for point in self.traj.points:
            points.append([point.x, point.y])

        return np.array(points)


class visualizer:
    def __init__(self):
        rospy.init_node("corridor_plotter")
        rospy.on_shutdown(self.shutdown)
        
        self.obs_cb = rospy.Subscriber("/paddedObs", Marker, self.obs_handle)
        self.traj_cb = rospy.Subscriber("/MINCO_path", Marker, self.traj_handle)
        self.helper_cb = rospy.Subscriber("/helperPolys", PoseArray, self.helper_handle)
        self.odom_cb = rospy.Subscriber("/gmapping/odometry", Odometry, self.odom_handle)
        self.corridor_cb = rospy.Subscriber("/polyCorridor", PoseArray, self.corridor_handle)

        # 
        self.curr_odom = None
        self.curr_obs = None
        self.curr_helper = None
        self.curr_corridor = None
        self.curr_traj = None

        self.data = []

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            if len(self.data) > 50:
                rospy.signal_shutdown("data size is larger than 1")

            if self.curr_odom is None or self.curr_obs is None or self.curr_helper is None or self.curr_corridor is None or self.curr_traj is None:
                continue

            dat = datum(self.curr_odom, self.curr_obs, self.curr_corridor, self.curr_helper, self.curr_traj)
            self.data.append(dat)

            self.curr_traj = None
            self.curr_helper = None
            self.curr_corridor = None

    def obs_handle(self, msg):
        self.curr_obs = msg

    def odom_handle(self, msg):
        self.curr_odom= msg

    def traj_handle(self, msg):
        self.curr_traj = msg

    def helper_handle(self, msg):
        self.curr_helper = msg

    def corridor_handle(self, msg):
        self.curr_corridor = msg

    def shutdown(self):

        cwd = os.getcwd()
        fpath = os.path.join(cwd, "data/")
        if not os.path.exists(fpath):
            os.makedirs(fpath)


        for i, d in enumerate(self.data):
            obs = d.clean_obs()
            plt.scatter(obs[:,0], obs[:,1],color='#808080', s=1)

            traj_points = d.clean_traj()
            for point in traj_points:
                plt.plot(traj_points[:,0], traj_points[:,1], color='black')

            corridor_polys = d.clean_corridor(False)
            for hPoly, vPoly in corridor_polys:
                verts = vPoly.vertices()
                plt.plot(np.append(verts[0], verts[0][0]), np.append(verts[1], verts[1][0]),color='b')


            cents = []
            for j in range(len(corridor_polys)-1):
                cents.append(corridor_polys[j][0].Intersection(corridor_polys[j+1][0]).ChebyshevCenter())

            cents = np.array(cents)
            plt.scatter(cents[:,0], cents[:,1],s=10,color="blue",alpha=.5)

            helper_polys = d.clean_corridor(True)
            for hPoly, vPoly in helper_polys:
                verts = vPoly.vertices()
                plt.plot(np.append(verts[0], verts[0][0]), np.append(verts[1], verts[1][0]),color='r')

            plt.savefig(os.path.join(fpath,f'corridor_{i}'))
            plt.clf()

        rospy.loginfo("shutting down!")

if __name__ == "__main__":
    vis = visualizer()
    vis.start()
