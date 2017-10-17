#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion

import math

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

#LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 100

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		# TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below

		rate = rospy.Rate(1)

		rospy.spin()

	def pose_cb(self, msg):
		# TODO: Implement
		x = msg.pose.position.x
		y = msg.pose.position.y

		#rospy.loginfo (" pose x:%s , y:%s", x, y)
		closest_wp = len(self.waypoints) + 1
		closest_dist = 999999

		for i in range(len(self.waypoints)):
			target_x = self.waypoints[i].pose.pose.position.x - x
			target_y = self.waypoints[i].pose.pose.position.y - y

			dist = math.sqrt((target_x * target_x) + (target_y * target_y))

			if (dist < closest_dist):
				closest_wp = i
				closest_dist = dist

		waypoint_x = self.waypoints[closest_wp].pose.pose.position.x
		waypoint_y = self.waypoints[closest_wp].pose.pose.position.y

		
		heading = math.atan2(waypoint_y - y, waypoint_x - x)


		(roll, pitch, yaw) = euler_from_quaternion(
									[msg.pose.orientation.x,
									msg.pose.orientation.y,
									msg.pose.orientation.z,
									msg.pose.orientation.w])
		angle = abs(yaw-heading)

		if (angle > math.pi / 4):
			closest_wp += 1

		final_waypoints = Lane()

		for i in range(closest_wp, closest_wp+LOOKAHEAD_WPS):
			self.set_waypoint_velocity(self.waypoints, i , 10./2.23693)
			final_waypoints.waypoints.append(self.waypoints[i])

		self.final_waypoints_pub.publish(final_waypoints)


	def waypoints_cb(self, lane):
		# TODO: Implement
		self.waypoints = lane.waypoints
		rospy.loginfo("waypoints size %s", len(self.waypoints))

		#rospy.logwarn("limiting waypoint velocity to 10 mph ")
		#for i in range(len(self.waypoints)):
		#	self.set_waypoint_velocity(self.waypoints, i, 10./2.23693)


	def traffic_cb(self, msg):
		# TODO: Callback for /traffic_waypoint message. Implement
		pass

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
