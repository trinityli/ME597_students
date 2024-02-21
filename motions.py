# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


#Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...




CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                        depth=10,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE)
        
        cmd_vel_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                        depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.VOLATILE)
        
        # Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist,'/cmd_vel', cmd_vel_qos)
 
        self.current_linear_vel = 0.1
        self.current_angular_vel = 0.0
        self.accel = 5
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "stamp"])

        self.imu_values = []
        self.odom_values = []
        self.laser_values = []
        


        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.imu_subscription=self.create_subscription(Imu,'/imu',self.imu_callback,qos_profile)
        
        # ENCODER subscription

        self.odom_subscription=self.create_subscription(Odometry,'/odom',self.odom_callback,qos_profile)
        
        # LaserScan subscription 
        
        self.laserscan_subscription=self.create_subscription(LaserScan,'/scan',self.laser_callback,qos_profile)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        # stamp = imu_msg.header.stamp.nanosec
        stamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        self.imu_initialized = True

        imu_data = [acc_x, acc_y, angular_z, stamp]

        # self.imu_values.append(imu_data)
        self.imu_logger.log_values(imu_data)
        return self.imu_values

        
    def odom_callback(self, odom_msg: Odometry):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        th = odom_msg.pose.pose.orientation.z
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        self.odom_initialized = True

        odom_data = [x, y, th, stamp]

        # self.odom_values.append(odom_data)
        self.odom_logger.log_values(odom_data)
        return self.odom_values
                
    def laser_callback(self, laser_msg: LaserScan):
        ranges = laser_msg.ranges
        stamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        self.laser_initialized = True

        laser_data = [ranges, stamp]

        # self.laser_values.append(laser_data)
        self.laser_logger.log_values(laser_data)
        return self.laser_values

                
    def timer_callback(self):

        i = 0

        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        # if self.imu_values:
        #     self.imu_logger.log_values([self.imu_values["acc_x"], self.imu_values["acc_y"], self.imu.values["angular_z"], self.imu_values["stamp"]])


        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self): # circle
        
        msg=Twist()
        # decrease angular velocity in z
        self.current_angular_vel = 5.0
        self.current_linear_vel = 2.0

        msg.angular.z = self.current_angular_vel
        msg.linear.x = self.current_linear_vel
        return msg

    def make_spiral_twist(self): # spiral
        msg=Twist()
        self.current_angular_vel = 5.0
        # self.current_linear_vel = 1.0

        self.radius_ += float(self.current_linear_vel/self.current_angular_vel)
        
        msg.angular.z = self.current_angular_vel
        msg.linear.x = self.current_linear_vel + self.radius_
        # msg.linear.x += self.radius_
        return msg
    
    def make_acc_line_twist(self): # line
        msg=Twist()
        self.current_linear_vel += self.accel * 0.1
        # print("linear vel is ", self.current_linear_vel)
        msg.linear.x = self.current_linear_vel
        return msg

import argparse

if __name__=="__main__":


    argParser=argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="circle")

    args = argParser.parse_args()

    rclpy.init()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
