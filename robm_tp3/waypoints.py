#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""robm_tp3/waypoints.py - Fait parcourir un chemin au robot

Ce noeud fait parcourir un chemin défini par des points de passage. On compare
la position actuelle au point visé pour décider de passer au suivant. La navigation
vers le point est réalosée par move.py.

S'abonne à :
	- odometry (nav_msgs/Odometry): pose du robot calculée par odométrie
Publie :
	- goal_pose (geometry_msgs/PoseStamped): but à rejoindre
"""

import rclpy
from rclpy.node import Node
from math import pi, cos, sin, atan2, sqrt

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry


# Fonctions utilitaires pour les angles / quaternions
def yaw_from_quaternion_msg( q ):
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

def quaternion_msg_from_yaw(yaw):
    """Construit un quaternion à partir d'un angle de cap"""
    ak = yaw / 2.0
    ck = cos(ak); sk = sin(ak)
    return Quaternion(x=0.0, y=0.0, z=sk, w=ck)


# Le noeud de publication des waypoints
class Waypoints(Node):
	def __init__(self):
		"""Constructeur de la classe Waypoints"""
		super().__init__('waypoints')


		# Liste des waypoints
		self.waypoints = [ (0.2,0.3,0.3), (0.1,0.15,0.3)]  # TODO

		# subscribers et publishers
		self._sub_odom = self.create_subscription(Odometry, "odometry", self.odom_callback, 10)
		self._goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

	
	def odom_callback(self, odom_msg:Odometry):
		"""Choisit le prochain point de passage en fonction de la position actuelle"""
		# Position actuelle du robot
		x = odom_msg.pose.pose.position.x
		y = odom_msg.pose.pose.position.y

		# TODO
		xGoal,yGoal,aGoal = self.listCoord[self.listCoord.count-1]

		dist = sqrt((self.xGoal-x)**2 + (self.yGoal-y)**2)

		if(dist>0.1):
			goal = PoseStamped()
			goal.pose.position.x = xGoal # TODO
			goal.pose.position.y = yGoal# TODO
			goal.pose.orientation = quaternion_msg_from_yaw(aGoal) # TODO
		else:
			self.listCoord = self.waypoints.pop()
			
			goal.pose.position.x = xGoal # TODO
			goal.pose.position.y = yGoal# TODO
			goal.pose.orientation = quaternion_msg_from_yaw(aGoal) # TODO


		
		self._goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
