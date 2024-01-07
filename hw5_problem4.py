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
 
    
    wp_list=[[7, 13], [6.949763183520403, 12.654925666151989], [6.452684207503364, 12.7088931712069], [6.035663100563276, 12.984743486353475], [5.908133897354216, 12.50128066220992], [5.524077736042422, 12.181123099502978], [5.567189309974393, 11.682985174633663], [6.0261883159046405, 11.484692682201286], [6.481187685621041, 11.277387174417056], [6.9014100463078325, 11.006438530613806], [7.390682859983088, 10.903423416197043], [7.639829815249237, 11.336927504622524], [8.124508110139242, 11.214098556639906], [8.57915169003663, 11.422183191483729], [8.920557933797559, 11.056886357770304], [9.366103037086159, 10.82997324458477], [9.40079815270448, 10.331178448148167], [9.748025104742945, 9.971409850605138], [10.236885566705896, 9.866455342701139], [10.600180323840265, 9.522919435642386], [10.319351051860561, 9.10923486982037], [10.05732843347864, 8.68338965444713], [9.734189631025096, 8.301838459521875], [10.026107630998434, 7.895902650197831], [9.61426218085123, 7.612383117902353], [9.35842183953329, 7.182795269198132], [8.977693914159925, 6.858686881708865], [8.572363998105384, 6.565928177053958], [8.076735087639486, 6.499958615054608], [7.576803042771634, 6.508201825245314], [7.098930452814615, 6.3611039402579], [6.660032043404564, 6.600620507988249], [6.166128705234862, 6.522777765585213], [5.682579214386, 6.395577569438325], [5.245257888253376, 6.153193457521716], [4.745534693813047, 6.169828629247692], [4.422331487758046, 5.788331987590431], [3.9530558504561033, 5.615762070644125], [3.5666456529924173, 5.298449670482904], [3.4684875816685654, 4.808179343995586], [3.5839922958146584, 4.321703588165936], [3.454218304217977, 3.8388384816350696], [3.192424594999578, 3.412852503211281], [2.8036364516823773, 3.0984581972578114], [2.930827684557843, 2.614906348662835], [2.962551936873039, 2.1159137917894464], [2.5953652290418594, 1.7765409561173793], [2.2155728211886574, 1.4513368249365668], [1.716946256065692, 1.4883712488374907], [1.218439295162422, 1.449760240938374], [1, 1]]
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
    