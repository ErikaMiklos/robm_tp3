#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""robm_tp3/obstacle_avoid.py - Transforme (v,w) pour éviter les obstacles

Ce noeud permet d'éviter les obstacles. Il prend en entrée une commande 
en vitesse, la distance à l'obstacle, et calcule une nouvelle commande 
permettant d'éviter / contourner l'obstacle.

S'abonne à :
	- cmd_vel_desired (geometry_msgs/Twist): vitesse désirée
	- range (sensor_msgs/Range): distance à l'obstacle (sonar)
Publie :
	- cmd_vel (geometry_msgs/Twist): commande en vitesse modifiée
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class ObstacleAvoid(Node):
	def __init__(self):
		"""Constructeur de la classe ObstacleAvoidance"""
		super().__init__('obstacle_avoid')
		# distance
		self.range_msg = None
		# vitesse désirée
		self.vel_msg = None
		# subscribers et publishers
		self._sub_vel = self.create_subscription(Twist, "cmd_vel_desired", self.vel_callback, 10)
		self._sub_range = self.create_subscription(Range, "range", self.range_callback, 10)
		self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
	
	def update_speed(self):
		"""Appelé à chaque mise à jour de la distance à l'obstacle ou de la viytesse désirée
		Publie la commande corrigée de vitesse lineaire v et angulaire w"""
		# On doit avoir au moins reçu un message de vitesse et un message de distance
		if self.vel_msg == None or self.range_msg == None:
			return 
		
		# Vitesse désirée (v_d, w_d) et distance mesurée d
		v_d = self.vel_msg.linear.x
		w_d = self.vel_msg.angular.z
		d = self.range_msg.range
		
		# TODO: Calcul de la vitesse a appliquer pour éviter l'obstacle
		v = v_d
		w = w_d

		
		# On publie la nouvelle commande de vitesse
		vel_msg = Twist()
		vel_msg.linear.x = v
		vel_msg.angular.z = w
		self._cmd_pub.publish(vel_msg)	


	def vel_callback(self, vel_msg:Twist):
		"""Callback du mise à jour de la vitesse désirée"""
		self.vel_msg = vel_msg
		self.update_speed()


	def range_callback(self, range_msg:Range):
		"""Callback de reception de message de distance"""
		self.range_msg = range_msg
		self.update_speed()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
