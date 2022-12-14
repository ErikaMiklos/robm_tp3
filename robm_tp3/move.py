#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""robm_tp3/move.py - Navigation vers un point

Ce noeud permet de faire naviguer le robot jusqu'à un but spécifié dans
RViz.

S'abonne à :
    - /goal_pose (geometry_msgs/PoseStamped): but
    - odometry (nav_msgs/Odometry): pose du robot calculée par odométrie
Publie :
    - cmd_vel (geometry_msgs/Twist): commande en vitesse
"""

import rclpy
from rclpy.node import Node
from math import pi, cos, sin, atan2, sqrt

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry


def yaw_from_quaternion_msg( q ):
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

def sawtooth( angle ):
    """Wrap an angle value within the interval [-pi; +pi["""
    return (angle + pi) % (2*pi) - pi


class MoveNode(Node):
    def __init__(self):
        super().__init__('move')

        # Position désirée
        self.x_d     = None
        self.y_d     = None
        # Cap désiré
        self.theta_d = None
        # Publishers et subscribers
        self._sub_goal = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 1)
        self._sub_odom = self.create_subscription(Odometry, "odometry", self.odom_callback, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        # compteur pour arrêter l'envoi de commandes après arrêt du robot
        self._nb_successive_zeros = 0
        
    
    def publish_speed_cmd(self, v, w):
        """Publie la commande de vitesse lineaire v et angulaire w
            
        Arrête de publier des commandes d'arrêt si le robot est déjà à 
        l'arrêt depuis un certain temps, pour éviter de fatiguer les 
        moteurs. Les vitesses sont en unités arbitraires, entre -1 et 1
            
        :param v: commande de vitesse linéaire (entre -1 et 1)
        :param w: commande de vitesse angulaire (entre -1 et 1)
        """
        # Mise à jour du compteur de commandes d'arrêt
        if v == 0 and w == 0:
            self._nb_successive_zeros += 1
        else:
            self._nb_successive_zeros = 0
            
        # Si nécessaire, on publie la commande
        if self._nb_successive_zeros < 5: 
            vel_msg = Twist()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            self._cmd_pub.publish(vel_msg)	


    def goal_callback(self, goal):
        """Callback du mise à jour du but"""
        self.get_logger().info(f"Goal set to x={goal.pose.position.x:.02f} y={goal.pose.position.y:.02f}")
        # Extract goal position and orientation from msg
        self.x_d      = goal.pose.position.x
        self.y_d      = goal.pose.position.y
        self.theta_d = yaw_from_quaternion_msg(goal.pose.orientation)


    def odom_callback(self, odom):
        """Callback de reception de message d'odometrie.
        Le calcul de la commande est effectué dans cette fonction.
        """
        # Ne rien faire si le but n'est pas connu
        if self.x_d == None or self.y_d == None or self.theta_d == None:
            return
        
        # Extract current position and orientation from msg
        x     = odom.pose.pose.position.x
        y     = odom.pose.pose.position.y
        theta = yaw_from_quaternion_msg(odom.pose.pose.orientation)

        # TODO: Calculer la distance au but
        d = sqrt((self.x_d-x)**2 + (self.y_d-y)**2)
        
        # TODO: Calculer le cap à suivre
        self.theta_d = atan2((self.x_d-x),(self.y_d-y))
        
        # TODO: Calculer la vitesse angulaire du robot (cf. control_heading.py)
        w = 0.3  # TODO
        
        # TODO: Calculer la vitesse linéaire du robot
        v = 0.1  # TODO
        
        # Publish speed command (in not zero for too long)
        self.publish_speed_cmd(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()