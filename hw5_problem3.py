#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np
import astar
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import some_python_module
from unmanned_systems_ros2_pkg import PIDTemplate

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 
    

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
                # wrap yaw from 0 to 2pi
        # if self.orientation_euler[2] < 0:
        #      self.orientation_euler[2] += 2*np.pi
        # else:
        #      self.orientation_euler[2] = self.orientation_euler[2]
        #print("yaw is", np.degrees(self.orientation_euler[2]))
        #print("yaw is", np.degrees(self.orientation_euler[2]))
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main()->None:
    rclpy.init(args=None)
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    
    cmd_vel = 1.0 #m/s
    ang_vel = 0.5 #rad/s
    stop_vel = 0.0
    time_duration_forward = 5
    time_duration_turnright = 2
    forward_speed=1.5
    turnright_speed=-0.15

    # time intilization ref 
    time_origin = get_time_in_secs(turtlebot_node)
    print("time now is", time_origin)

    kp_angular = 2
    ki_angular = 0.0
    kd_angular = 0.0
    dt_angular = 1/20

    kp_linear = 10
    ki_linear = 0.0
    kd_linear = 0.0
    dt_linear = 1/20

    pid_angular = PIDTemplate.PID(
        kp=kp_angular,
        ki=ki_angular,
        kd=kd_angular,
        dt=dt_angular
    )
    pid_linear = PIDTemplate.PID(
        kp=kp_linear,
        ki=ki_linear,
        kd=kd_linear,
        dt=dt_linear   
    )
    # set a desired headind angle
    des_yaw_angle_rad = np.deg2rad(180)
    distance_error_tolerance_m=0.15
    max_angular_speed = 2.3 #rad/s
    # set the map, starting point,goal point and obstacle list
    Start_x = 1
    Start_y = 1
    Goal_x = 7
    Goal_y = 13
    Max_x = 15
    Max_y = 15
    min_x=0
    min_y=0
    Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
    8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
    5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
    Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
    8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
    14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
    gs=1
    obstacle_dia = 0.5
    robot_radius = 0.5
    Obstacle_list=[]
    
    for x,y in zip(Obstacle_x,Obstacle_y):
         obs=(x,y)
         Obstacle_list.append(obs)
    
    wp_list=astar.dijkstras(Start_x,Start_y,min_x,Max_x,min_y,Max_y,Goal_x,Goal_y,gs,Obstacle_list,obstacle_dia,robot_radius)
    wp_list = wp_list[::-1]
  
    print("waypoints are ", wp_list)
    heading_error_tolerance = np.deg2rad(1)
    distance_error_tolerance = 0.3
    try:

        rclpy.spin_once(turtlebot_node)

        while rclpy.ok():
             
         #current_wp in wp_list:
             for wp in wp_list:
                current_wp = wp


                ## sent direct heading
                dy = current_wp[1] - turtlebot_node.current_position[1]
                dx = current_wp[0] - turtlebot_node.current_position[0]

                desired_heading_rad = np.arctan2(dy,dx)
                # if desired_heading_rad<0:
                #      desired_heading_rad = desired_heading_rad+2*math.pi
                current_heading_error_rad=pid_angular.compute_error(
                    desired_heading_rad,
                    turtlebot_node.orientation_euler[2]
                )
                print("current heading error is",current_heading_error_rad)


                current_distance_error = np.sqrt(dx**2 + dy**2)
                
                print("desired heading is",np.rad2deg(desired_heading_rad))


                while abs(current_heading_error_rad) >= heading_error_tolerance:
                    dy = current_wp[1] - turtlebot_node.current_position[1]
                    dx = current_wp[0] - turtlebot_node.current_position[0]

                    desired_heading_rad = np.arctan2(dy,dx)
                
                    current_heading_error_rad=pid_angular.compute_error(
                            desired_heading_rad,
                            turtlebot_node.orientation_euler[2]
                        ) 


                    if(abs(current_heading_error_rad) < heading_error_tolerance):
                            print("I am done")
                            turtlebot_node.move_turtle(0.0,0.0)
                            break
                    
                    angular_gains = pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2])

                        
                    print("my error is",np.rad2deg(pid_angular.error[0]),"current yaw is",turtlebot_node.orientation_euler[2])
                    if angular_gains >= max_angular_speed:
                                angular_gains = max_angular_speed
                    elif angular_gains <= -max_angular_speed:
                                angular_gains = -max_angular_speed
                        
                    turtlebot_node.move_turtle(0.0,angular_gains)
                
                    rclpy.spin_once(turtlebot_node)

                    
                if abs(current_heading_error_rad) < heading_error_tolerance:
                    turtlebot_node.move_turtle(0.0,0.0)
                                
                ### once the heading is send forward
                while current_distance_error >= distance_error_tolerance_m:

                            current_heading_error_rad = pid_angular.compute_error(
                                desired_heading_rad,
                                turtlebot_node.orientation_euler[2]                
                            )



                            angular_gains = pid_angular.get_gains(
                                desired_heading_rad,
                                turtlebot_node.orientation_euler[2]
                            )

                            print("my heading error is", 
                                np.rad2deg(pid_angular.error[0]))

                            if angular_gains >= max_angular_speed :
                                angular_gains = max_angular_speed 
                            elif angular_gains <= -max_angular_speed :
                                angular_gains = -max_angular_speed 
                            

                            dx = current_wp[0] - turtlebot_node.current_position[0]
                            dy = current_wp[1] -  turtlebot_node.current_position[1]
                            current_distance_error = np.sqrt(dx**2 + dy**2)

                            if (current_distance_error <= distance_error_tolerance_m):
                                print("converged to wp")
                                turtlebot_node.move_turtle(0.0, 0.0)
                                break

                            turtlebot_node.move_turtle(0.4, angular_gains)
                            print("the y position is ", turtlebot_node.current_position[1])

                            rclpy.spin_once(turtlebot_node)

        
                #    while current_distance_error > distance_error_tolerance:
                #         if current_distance_error <= distance_error_tolerance:
                #              break
                #         dy = current_wp[1] - turtlebot_node.current_position[1]
                #         dx = current_wp[0] - turtlebot_node.current_position[0]

                #         current_distance_error = np.sqrt(dx**2 + dy**2) 
                #         turtlebot_node.move_turtle(1.1,0.0)

             
             #rclpy.spin_once(turtlebot_node)

         

        
             
    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0,0.0)

if __name__ == '__main__':
    """apply imported function"""
    main()
    