#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS =50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.3 # comfort decel 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        # a subscriber for /traffic_waypoint and /obstacle_waypoint below
     
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32,self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
       # member variables(store important info)
        self.base_waypoints = None
        # this car's position
        self.pose = None
        # cars' 2d waypoints(x,y)
        self.waypoints_2d = None
        # kd tree for getting closest waypoint's index
        self.waypoint_tree = None
        # stop line waypoint index 
        self.stopline_wp_idx = -1
        
        rospy.loginfo('waypoint_updater is initialized')
        self.loop()
        
    # this function is for publish final waypoints at 50hz rate
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                # get closest waypoint index and publish its index and next 199 indices
                #closest_waypoint_idx = self.get_closest_waypoint_idx()
                #rospy.loginfo("LOOP working and closest waypoint idx is:{0}".format(closest_waypoint_idx))
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        #Check if closest is ahead or behind car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord =self.waypoints_2d[closest_idx-1]
        
        # equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val> 0:
            # the closest coordiate is behind the car so we need take next inx
            closest_idx = (closest_idx + 1 ) % len(self.waypoints_2d)
        
        return closest_idx


    def publish_waypoints(self):
        lane = Lane()
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        closest_waypoint_idx = self.get_closest_waypoint_idx()
        ub_idx = closest_waypoint_idx + LOOKAHEAD_WPS 
        base_waypoints = self.base_waypoints.waypoints[closest_waypoint_idx:ub_idx]
        # initial state or stopline index is greater than upper bound index -use base_waypints
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= ub_idx):
            lane.waypoints = base_waypoints
        else:
            # need to decelerate
            lane.waypoints = self.decel_waypoints(base_waypoints,closest_waypoint_idx)
        return lane
    
    def decel_waypoints(self,waypoints,closest_idx):
        waypoints_list = []
        for i, wp  in enumerate(waypoints):
            new_wp = Waypoint()
            new_wp.pose = wp.pose
            stopline_idx = max(self.stopline_wp_idx - closest_idx -2 ,0)
            dist = self.distance(waypoints, i, stopline_idx)
            # reduce speed gradually 
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.:
                vel = 0.0
            new_wp.twist.twist.linear.x =min(vel,self.get_waypoint_velocity(wp))
            waypoints_list.append(new_wp)
        return waypoints_list

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d =[[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # add waypoints to KD tree for getting closest waypoint purpose
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        #  Callback for /traffic_waypoint message. 
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
