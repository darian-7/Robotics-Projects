# version 0.0
# Jose Cuaran


import math
import numpy as np
import rclpy
from rclpy.node import Node
#from rclpy.clock import Clock

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterEvent
from tf2_msgs.msg import TFMessage
from mobile_robotics.utils import quaternion_from_euler, lonlat2xyz #edit according to your package's name


class OdometryNode(Node):
    # Initialize some variables
    
    gyro_yaw = 0.0
    blspeed = 0.0 #back left wheel speed
    flspeed = 0.0 #front left wheel speed


    x = 0.0 # x robot's position
    y = 0.0 # y robot's position
    theta = 0.0 # heading angle
    l_wheels = 0.3 # Distance between right and left wheels

    odom = 0.0
    rosout = 0.0
    tf = 0.0


    last_time = 0.0
    current_time = 0.0

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Declare subscribers to all the topics in the rosbag file, like in the example below. Add the corresponding callback functions.
        self.subscription_gyro_yaw = self.create_subscription(Float32, 'gyro_yaw', self.callback_Gy, 10)
        # your code here
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.callback_odom, 10)
        self.subscription_parameter_events = self.create_subscription(ParameterEvent, 'parameter_events', self.callback_para, 10)
        self.subscription_rosout = self.create_subscription(Log, 'rosout', self.callback_rosout, 10)
        self.subscription_tf = self.create_subscription(TFMessage, 'tf', self.callback_tf, 10)
        # ---------
        self.last_time = self.get_clock().now().nanoseconds/1e9
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) #keep in mind how to declare publishers for next assignments
        self.timer = self.create_timer(0.1, self.timer_callback_odom) #It creates a timer to periodically publish the odometry.
        
        self.tf_broadcaster = TransformBroadcaster(self) # To broadcast the transformation between coordinate frames.


        self.file_object_results  = open("results_part1.txt", "w+")
        self.timer2 = self.create_timer(0.1, self.callback_write_txt_file) #Another timer to record some results in a .txt file
        
    def callback_Gy(self, msg):
        self.gyro_yaw = msg.data

    def callback_para(self, msg):
        self.odom = msg.data

    def callback_rosout(self, msg):
        self.rosout = msg.data

    def callback_tf(self, msg):
        self.tf = msg.data

    def callback_bl(self, msg):
        self.blspeed = msg.data

    def callback_fl(self, msg):
        self.flspeed = msg.data
        

    def timer_callback_odom(self):
        '''
        Compute the linear and angular velocity of the robot using the differential-drive robot kinematics
        Perform Euler integration to find the position x and y of the robot
        '''

        self.current_time = self.get_clock().now().nanoseconds/1e9
        dt = self.current_time - self.last_time # DeltaT
        
        vl = (self.blspeed + self.flspeed)/2.0  #Average Left-wheels speed
        vr = (self.blspeed + self.flspeed)/2.0  # ... Your code here. Average right-wheels speed
        
        v = (vr + vl) / 2.0     # ... Linear velocity of the robot
        w = (vr - vl) / self.l_wheels    # ... Angular velocity of the robot

        change_orien = w * dt      # Change in orientation
        change_x = v * math.cos(self.theta + change_orien / 2.0) * dt
        change_y = v * math.sin(self.theta + change_orien / 2.0) * dt

        self.x = change_x    # ...Position
        self.y = change_y # ...Position
        self.theta = change_orien # ...Heading angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize theta between -pi and pi

        position = [self.x, self.y, 0.0]
        quater = quaternion_from_euler(0.0, 0.0, self.theta)
        print("position: ", position)
        print("orientation: ", quater)


        # We need to set an odometry message and publish the transformation between two coordinate frames
        # Further info about odometry message: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        # Further info about tf2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
        # Further info about coordinate frames in ROS: https://www.ros.org/reps/rep-0105.html

        frame_id = 'odom'
        child_frame_id = 'base_link'
        
        
        self.broadcast_tf(position, quater, frame_id, child_frame_id)  # Before creating the odometry message, go to the broadcast_tf function and complete it.
        
        odom = Odometry()
        odom.header.frame_id = frame_id
        odom.header.stamp = self.get_clock().now().to_msg()

        # set the pose
        odom.pose.pose.position.x = position[1] # x_coord of robot current pos in odom frame
        odom.pose.pose.position.y = position[2] # y_coord of robot current pos in odom frame
        odom.pose.pose.position.z = 0.0 # robot moves in 2D
        # quaternions
        odom.pose.pose.orientation.x = quater[0]
        odom.pose.pose.orientation.y = quater[1]
        odom.pose.pose.orientation.z = quater[2]
        odom.pose.pose.orientation.w = quater[3]

        # set the velocities
        odom.child_frame_id = child_frame_id
        odom.twist.twist.linear.x = v # robot's forward v in robot frame
        odom.twist.twist.linear.y = 0.0 # no lateral movement
        odom.twist.twist.linear.z = 0.0 # 2D operation
        odom.twist.twist.angular.x = 0.0 # no roll
        odom.twist.twist.angular.y = 0.0 # no pitch
        odom.twist.twist.angular.z = w # angular v (yaw rate)

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        
    def broadcast_tf(self, pos, quater, frame_id, child_frame_id):
        '''
        It continuously publishes the transformation between two reference frames.
        Complete the translation and the rotation of this transformation
        '''
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        # Uncomment next lines and complete the code
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = 0.0     # 2D

        t.transform.rotation.x = quater[0]
        t.transform.rotation.y = quater[1]
        t.transform.rotation.z = quater[2]
        t.transform.rotation.w = quater[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    
    def callback_write_txt_file(self):
        if (self.x != 0 or self.y != 0 or self.theta != 0):
            self.file_object_results.write(str(self.current_time) + " " + str(self.x)+" "+str(self.y)+" "+str(self.theta)+"\n")

    
def main(args=None):
    rclpy.init(args=args)

    odom_node = OdometryNode()

    rclpy.spin(odom_node)
    odom_node.file_object_results.close()
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
