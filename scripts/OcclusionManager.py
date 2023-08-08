#! /usr/bin/env python
import sys
import time
import math
import copy
import rospy
import random
import tf2_ros
import actionlib
import numpy as np

from collections import deque
from sklearn.neighbors import KDTree
from sensor_msgs.msg import LaserScan
from scipy.spatial.distance import cdist
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, MapMetaData, OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OcclusionState:

    default = 0
    nearby = 1
    horizon = 2
    goal = 3
    free_space = 4
    centroid = 5
    free_centroid = 6
    frontier = 7
    invalid = 8

    colors = [
        (0, 0, 0, 0),
        (0, 0, 1, 0.5),
        (0, 1, 0, 0.5),
        (1, 1, 0, 0.5),
        (1, 0, 1, 0.5),
        (0, 1, 1, 0.5),
        (1, 0, 0, 0.5),
        (1, 0.5, 0.5, 0.5),
        (1, 0, 0, 1),
    ]


class ManagerState:

    WAITING_FOR_GOAL = 1
    GOING_TO_GOAL = 2
    MOVE_TO_DEFAULT_GOAL = 3
    AVOIDING_OBSTACLE = 4
    ROTATING = 5


class Occlusion:
    def __init__(
        self, mx, my, color, iden, shape, scale, pointA, pointB, state, time_created
    ):

        self.mx = mx
        self.my = my
        self.color = color
        self.iden = iden
        self.shape = shape
        self.scale = scale
        self.pointA = pointA
        self.pointB = pointB

        # If counter reaches theta, we know it is a valid occlusion
        self.counter = 0
        # self.is_valid = False
        self.is_valid = True
        # self.threshold = 3
        self.threshold = 0

        self.max_priority = 20
        self.priority = 0

        self.is_goal = False
        self.state = state
        self.initial_state = state

        self.times_goal = 0
        self.times_failed = 0

        self.time_created = time_created

    def set_goal(self):
        self.is_goal = True
        self.state = OcclusionState.goal
        self.color = OcclusionState.colors[self.state]

    def unset_goal(self):
        self.is_goal = False
        self.state = self.initial_state
        self.color = OcclusionState.colors[self.state]

    def get_coords(self):
        return (self.mx, self.my)

    def updateCounter(self):

        if self.is_valid:
            return

        self.counter += 1

        if self.counter > self.threshold:
            self.is_valid = True

    def updatePriority(self):

        if self.priority < self.max_priority:
            self.priority += 1
        else:
            return

        if self.color == (1, 1, 0, 1) or (1, 0, 1, 0.5):
            return

        if self.color[1] != 0:
            self.color = (
                self.priority / self.max_priority,
                1 - (self.priority / self.max_priority),
                0,
                1,
            )
        elif self.color[2] != 0:
            self.color = (
                self.priority / self.max_priority,
                0,
                1 - (self.priority / self.max_priority),
                1,
            )

    def __str__(self):
        return "({},{}): id = {}".format(
            round(self.mx, 2), round(self.my, 2), self.iden
        )


class OcclusionManager:
    def __init__(
        self,
        default_goals,
        jackal_width,
        jackal_length,
        publisher,
        pose_arr_pub,
        autonomous=True,
    ):
        self.occlusion_buffer = []
        self.publish_buffer = []
        self.delete_buffer = []

        # parameters about occlusions
        self.epsilon = 0.5
        self.buffer_len = 100
        self.threshold = 1

        self.counter = 0
        self.max_counter = 30
        self.is_updated = False
        self.state = ManagerState.WAITING_FOR_GOAL

        self.default_goals = default_goals
        self.deleted_goals = []

        self.jackal_width = jackal_width
        self.jackal_length = jackal_length

        self.occupancy_grid = None
        self.publisher = publisher
        self.pose_arr_pub = pose_arr_pub

        self.current_default_goal = 0
        self.current_goal = [float("inf"), float("inf")]
        self.current_goal_occ = None

        self.autonomous = autonomous
        self.jackalpos = None
        self.ranges = [float("inf")] * 720

        self.timespan = 30
        self.freq = 5
        self.previous_positions = deque(maxlen=self.timespan * self.freq)
        self.max_attempts = self.timespan * self.freq
        self.positions_to_try = None

        self.visited = []

    def update_map(self, data):

        self.occupancy_grid = data

    def update_lidar(self, data):

        self.ranges = data.ranges

    def updateBuffer(self, occlusions, jackal_pos):

        # self.previous_positions.append((jackal_pos.position.x, jackal_pos.position.y))
        self.jackalpos = jackal_pos

        is_goal_valid = True

        # Update occlusion buffer based on some things
        for ind, occ in reversed(list(enumerate(self.occlusion_buffer))):
            # If the robot is inside any buffered occlusion, remove it.
            if (
                cal_dist(jackal_pos.position.x, jackal_pos.position.y, occ.mx, occ.my)
                <= occ.scale * 3 / 4
            ):  # 2*(JACKAL_LENGTH**2 + JACKAL_WIDTH**2):

                if occ.state == OcclusionState.free_space:
                    continue

                # Mark that the goal has just been deleted
                is_goal_valid = not occ.is_goal
                print(
                    cal_dist(
                        jackal_pos.position.x, jackal_pos.position.y, occ.mx, occ.my
                    )
                )
                rospy.loginfo("Occ is too close to jackal")
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            # If obstacle is too close to occlusion, remove it.
            elif overlapping_obstacle(occ, self.occupancy_grid):

                if occ.state == OcclusionState.free_centroid:
                    continue

                # Mark that the goal has just been deleted
                is_goal_valid = not occ.is_goal
                rospy.loginfo("Occlusion is too close to obstacle")
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            elif (
                occ.state == OcclusionState.centroid
                and free_space_amount(occ.mx, occ.my, occ.scale, self.occupancy_grid)
                > 0.5
            ):

                is_goal_valid = not occ.is_goal
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            elif (
                occ.state == OcclusionState.frontier
                and free_space_amount(occ.mx, occ.my, occ.scale, self.occupancy_grid)
                > .70 #0.90
            ):

                is_goal_valid = not occ.is_goal
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            elif (
                occ.state == OcclusionState.nearby
                and free_space_amount(occ.mx, occ.my, occ.scale, self.occupancy_grid)
                > .60 #0.95
            ):

                rospy.loginfo("Occlusion is in free space")
                is_goal_valid = not occ.is_goal
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            # If the occlusion has been targeted as a goal too many times, there is likely
            # an oscillation. Remove the occlusion.

            elif occ.times_goal > 5:
                is_goal_valid = not occ.is_goal
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

            elif occ.times_failed > 1:
                is_goal_valid = not occ.is_goal
                self.delete_buffer.append(self.occlusion_buffer[ind])
                del self.occlusion_buffer[ind]

        # Now that occlusion buffer has been updated, it's time to check which new occlusions should be added to the buffer
        is_modified = False
        for occlusion in occlusions:
            x, y = occlusion.mx, occlusion.my

            if (
                cal_dist(jackal_pos.position.x, jackal_pos.position.y, x, y)
                <= occlusion.scale * 3 / 4
            ):

                continue

            # if new occlusion is overlapping an obstacle, ignore it
            if overlapping_obstacle(occlusion, self.occupancy_grid):

                if occlusion.state != OcclusionState.free_centroid:
                    # rospy.loginfo("Occlusion at (%.2f, %.2f) is overlapping obstacle, not adding",x,y)

                    continue

            # if occlusion is the centroid of a shadow and it's in free space, ignore it
            if (
                occlusion.state == OcclusionState.centroid
                and free_space_amount(x, y, occlusion.scale, self.occupancy_grid) > 0.5
            ):
                continue

            if (
                occlusion.state == OcclusionState.frontier
                and free_space_amount(x, y, occlusion.scale, self.occupancy_grid) > .70 #0.9
            ):
                continue

            if (
                occlusion.state == OcclusionState.nearby
                and free_space_amount(x, y, occlusion.scale, self.occupancy_grid) > .60 #0.95
            ):
                continue

            add_flag = True
            for occ in self.occlusion_buffer:
                if cal_dist(occ.mx, occ.my, x, y) < self.epsilon:# * cal_dist(
                #occ.mx, occ.my, jackal_pos.position.x, jackal_pos.position.y
                #):
                    add_flag = False
                    break

            # If occlusion is really close to another occlusion, ignore it. This threshold should be low enough that only noise fluctuations are discarded
            if not add_flag:
                continue

            # if midpoint of new sphere intersects any buffered spheres, remove them and plot this new one
            for ind, data in reversed(list(enumerate(self.occlusion_buffer))):

                if is_intersecting(data, occlusion):

                    # Do not add frontier/centroid if occlusion is already present there
                    if (
                        occlusion.state == OcclusionState.frontier
                        and data.state == OcclusionState.nearby
                    ):
                        add_flag = False
                        break

                    if (
                        occlusion.state == OcclusionState.centroid
                        and data.state == OcclusionState.nearby
                    ):
                        add_flag = False
                        break

                    occlusion.times_goal = data.times_goal
                    if data.is_goal:

                        is_goal_valid = False

                    if data.state == occlusion.state:
                        occlusion.color = data.color

                    self.delete_buffer.append(self.occlusion_buffer[ind])
                    del self.occlusion_buffer[ind]

                elif (
                    data.state == OcclusionState.centroid
                    and occlusion.state == OcclusionState.centroid
                    and cal_dist(data.mx, data.my, x, y) <= 2
                ):

                    occlusion.times_goal = data.times_goal
                    if data.is_goal:
                        is_goal_valid = False

                    self.delete_buffer.append(self.occlusion_buffer[ind])
                    del self.occlusion_buffer[ind]

            if add_flag:

                print("Adding to manager")
                is_modified = True
                occlusion.iden = self.counter
                self.occlusion_buffer.append(occlusion)

                if len(self.occlusion_buffer) > self.buffer_len:
                    rospy.loginfo(
                        "deleting "
                        + str(self.occlusion_buffer[0])
                        + " because buffer is full"
                    )
                    is_goal_valid = self.occlusion_buffer[0].is_goal
                    self.delete_buffer.append(self.occlusion_buffer[0])
                    del self.occlusion_buffer[0]

                self.counter += 1
                self.is_updated = True

    def publishOcclusions(self, debug=False):

        # if not self.is_updated and len(self.delete_buffer) == 0:
        #     return

        marker_arr = MarkerArray()
        marker_arr.markers = []

        for ind, occ in list(enumerate(self.delete_buffer)):
            marker_arr.markers.append(self.generateDeleteMsg(occ))

        if debug:
            msg = self.generatePublishMsg(Marker.DELETEALL, namespace="point")
            marker_arr.markers.append(msg)

        for ind, occ in list(enumerate(self.occlusion_buffer)):

            if occ.is_valid:
                marker_arr.markers.append(self.generatePublishMsg(Marker.ADD, occ))

            if debug and occ.state == OcclusionState.nearby:
                msg1, msg2 = self.plotOcclusionPoints(occ)
                marker_arr.markers.append(msg1)
                marker_arr.markers.append(msg2)

        self.publisher.publish(marker_arr)
        self.is_updated = False

        self.delete_buffer = []
        # sys.exit()
        # rospy.loginfo([str(i) for i in self.occlusion_buffer])

    def publishPoints(self):

        if not self.is_updated:
            return

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        for occ in self.occlusion_buffer:

            if occ.state == OcclusionState.nearby:
                p1 = Pose()
                p1.position.x = occ.pointA[0]
                p1.position.y = occ.pointA[1]
                p1.position.z = 0

                p2 = Pose()
                p2.position.x = occ.pointB[0]
                p2.position.y = occ.pointB[1]
                p2.position.z = 0

                msg.poses.append(p1)
                msg.poses.append(p2)

        self.pose_arr_pub.publish(msg)
        

    def publishShadows(self, line_list):
        if not self.is_updated:
            return

        marker_arr = MarkerArray()
        marker_arr.markers = [line_list]

        self.publisher.publish(marker_arr)


    def clear_occlusions(self):

        marker_arr = MarkerArray()
        for occ in self.occlusion_buffer:
            if occ.state == OcclusionState.frontier:
                marker_arr.markers.append(self.generateDeleteMsg(occ))

        self.publisher.publish(marker_arr)


    def generateDeleteMsg(self, occlusion):
        mx = occlusion.mx
        my = occlusion.my

        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.ns = "occlusion"
        msg.id = occlusion.iden
        msg.action = Marker.DELETE

        return msg

    def plotOcclusionPoints(self, occlusion):
        p1 = Occlusion(
            occlusion.pointA[0],
            occlusion.pointA[1],
            (1, 0, 0, 1),
            2 * occlusion.iden,
            occlusion.shape,
            0.25,
            None,
            None,
            OcclusionState.nearby,
            rospy.get_time(),
        )
        p2 = Occlusion(
            occlusion.pointB[0],
            occlusion.pointB[1],
            (1, 0, 0, 1),
            2 * occlusion.iden + 1,
            occlusion.shape,
            0.25,
            None,
            None,
            OcclusionState.nearby,
            rospy.get_time(),
        )

        msg1 = self.generatePublishMsg(Marker.ADD, p1, "point")
        msg2 = self.generatePublishMsg(Marker.ADD, p2, "point")

        return (msg1, msg2)

    def generatePublishMsg(self, action, occlusion=None, namespace=None):

        mx = my = 0
        if occlusion is not None:
            mx, my = occlusion.mx, occlusion.my

        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        if namespace is None:
            msg.ns = "occlusion"
        else:
            msg.ns = namespace
        # when counter > buffer_len, we are rewritting old spheres
        if occlusion is not None:
            msg.id = occlusion.iden
        msg.action = action

        if action == Marker.ADD:

            msg.type = occlusion.shape
            msg.pose.position.x = mx
            msg.pose.position.y = my
            msg.pose.position.z = 1
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1

            msg.lifetime = rospy.Duration()

            # Set scale of sphere to some percentage of distance between the two identified points
            msg.scale.x = msg.scale.y = msg.scale.z = occlusion.scale

            msg.color.r, msg.color.g, msg.color.b, msg.color.a = occlusion.color

        return msg


def cal_dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# Check if two spheres (occlusions) intersect one another
def is_intersecting(occ1, occ2):
    # if occ1.state != Occ
    return cal_dist(occ1.mx, occ1.my, occ2.mx, occ2.my) <= (occ1.scale + occ2.scale) / 2


def overlapping_obstacle(occlusion, occupancy_grid):
    mx, my = occlusion.mx, occlusion.my
    scale = occlusion.scale

    if occupancy_grid is None:
        return

    # if occlusion is outside occupancy grid don't bother checking
    OBSTACLE_PROB_THRESH = 80

    # Get grid coordinates of occlusion
    origin = occupancy_grid.info.origin
    resolution = occupancy_grid.info.resolution
    w, h = occupancy_grid.info.width, occupancy_grid.info.height

    # wi,hi = (mx-jackal_width - origin.position.x)/resolution, (my-jackal_length - origin.position.y)/resolution
    # wf,hf = (mx+jackal_width - origin.position.x)/resolution, (my+jackal_length - origin.position.y)/resolution
    wi, hi = (mx - scale / 2 - origin.position.x) / resolution, (
        my - scale / 2 - origin.position.y
    ) / resolution
    wf, hf = (mx + scale / 2 - origin.position.x) / resolution, (
        my + scale / 2 - origin.position.y
    ) / resolution

    # Check if width / height are NaN or inf
    if math.fabs(wi) == float("inf") or wi != wi:
        wi = w
    if math.fabs(hi) == float("inf") or hi != hi:
        hi = h

    if math.fabs(wf) == float("inf") or wf != wf:
        wf = w
    if math.fabs(hf) == float("inf") or hf != hf:
        hf = h

    wi, hi = int(min(wi, w)), int(min(h, hi))
    wf, hf = int(min(w, wf)), int(min(hf, h))

    if wf >= w or wi <= 0:
        return False

    if hf >= h or hi <= 0:
        return False

    for i in range(wi, wf):
        if i >= w:
            break
        for j in range(hi, hf):
            if j >= h:
                break
            if i + j * w >= w * h:
                break

            # If chance of obstacle being at point is too high, we are overlapping
            # the obstacle
            if occupancy_grid.data[i + j * w] > OBSTACLE_PROB_THRESH:
                return True

    return False


def free_space_amount(mx, my, scale, occupancy_grid):

    if occupancy_grid is None:
        return

    origin = occupancy_grid.info.origin
    resolution = occupancy_grid.info.resolution
    w, h = occupancy_grid.info.width, occupancy_grid.info.height

    radius = scale / 2

    wi, hi = int((mx - radius - origin.position.x) / resolution), int(
        (my - radius - origin.position.y) / resolution
    )
    wf, hf = int((mx + radius - origin.position.x) / resolution), int(
        (my + radius - origin.position.y) / resolution
    )
    wi, hi = max(0, wi), max(0, hi)
    wf, hf = min(w, wf), min(h, hf)

    if wf >= w or wi <= 0:
        return False

    if hf >= h or hi <= 0:
        return False

    area = 0.0
    count = 0
    for i in range(wi, wf):
        if i >= w:
            break
        for j in range(hi, hf):
            if j >= h:
                break

            # idx = i + j*w
            idx = i + j * w

            # if radius**2 < i**2 + j**2:
            #     continue
            if idx >= w * h:
                break

            if occupancy_grid.data[idx] < 0:
                count += 1
                continue

            count += 1
            # area += (1-occupancy_grid.data[i+j*w])*resolution**2
            area += (100 - occupancy_grid.data[i + j * w]) / 100

    # return area/(math.pi*radius**2) > .8
    if count == 0:
        return 0

    if wf == wi or hf == hi:
        return 1.0

    return area / count  # ((wf - wi) * (hf - hi))

