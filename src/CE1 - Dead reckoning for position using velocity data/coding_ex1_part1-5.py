# version 0.0
# Jose Cuaran

# Darian Irani - Mobile Robotics Coding Exercise 1


import math
import numpy as np
import rclpy
from rclpy.node import Node
#from rclpy.clock import Clock

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from utils import quaternion_from_euler, lonlat2xyz #edit according to your package's name


class OdometryNode(Node):
    # Initialize some variables
    
    gyro_yaw = 0.0
    gyro_pitch = 0.0
    gyro_roll = 0.0
    brspeed = 0.0
    blspeed = 0.0 #back left wheel speed
    flspeed = 0.0 #front left wheel speed
    frspeed = 0.0
    Accelx = 0.0
    Accely = 0.0
    Accelz = 0.0

    x = 0.0 # x robot's position
    y = 0.0 # y robot's position
    theta = 0.0 # heading angle
    l_wheels = 0.3 # Distance between right and left wheels

    last_time = 0.0
    current_time = 0.0

# Create a ROS node to read the messages from the rosbag file and determine the robot’s position
# along the trajectory. Publish an odometry ROS message which includes the position, 
# the linear and angular velocities, and the heading (in quaternions representation) of the robot.

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Declare subscribers to all the topics in the rosbag file, like in the example below. Add the corresponding callback functions.
        self.subscription_Gyro_yaw = self.create_subscription(Float32, 'Gyro_yaw', self.callback_Gyro_yaw, 10)
        # your code here
        self.subscription_Accelx = self.create_subscription(Float32, 'Accelx', self.callback_Accelx, 10)
        self.subscription_Accely = self.create_subscription(Float32, 'Accely', self.callback_Accelz, 10)
        self.subscription_Accelz = self.create_subscription(Float32, 'Accelz', self.callback_Accelz, 10)
        self.subscription_blspeed = self.create_subscription(Float32, 'Blspeed', self.callback_blspeed, 10)
        self.subscription_brspeed = self.create_subscription(Float32, 'Brspeed', self.callback_brspeed, 10)
        self.subscription_flspeed = self.create_subscription(Float32, 'Flspeed', self.callback_flspeed, 10)
        self.subscription_frspeed = self.create_subscription(Float32, 'Frspeed', self.callback_frspeed, 10)
        self.subscription_Gyro_pitch = self.create_subscription(Float32, 'Gyro_pitch', self.callback_Gyro_pitch, 10)
        self.subscription_Gyro_roll = self.create_subscription(Float32, 'Gyro_roll', self.callback_Gyro_roll, 10)
        self.subscription_latitude = self.create_subscription(Float32, 'latitude', self.callback_latitude, 10)
        self.subscription_longitude = self.create_subscription(Float32, 'longitude', self.callback_longitude, 10)
       
        # ---------
        self.last_time = self.get_clock().now().nanoseconds/1e9
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) #keep in mind how to declare publishers for next assignments
        self.timer = self.create_timer(0.1, self.timer_callback_odom) #It creates a timer to periodically publish the odometry.
        
        self.tf_broadcaster = TransformBroadcaster(self) # To broadcast the transformation between coordinate frames.


        self.file_object_results  = open("results_part1.txt", "w+")
        self.timer2 = self.create_timer(0.1, self.callback_write_txt_file) #Another timer to record some results in a .txt file
        
    def callback_Gyro_yaw(self, msg):
        self.Gyro_yaw = msg.data

    def callback_Accelx(self, msg):
        self.Accelx = msg.data

    def callback_Accely(self, msg):
        self.Accely = msg.data

    def callback_Accelz(self, msg):
        self.Accelz = msg.data

    def callback_blspeed(self, msg):
        self.Blspeed = msg.data

    def callback_brspeed(self, msg):
        self.Br = msg.data

    def callback_flspeed(self, msg):
        self.Fl = msg.data

    def callback_frspeed(self, msg):
        self.Fr = msg.data

    def callback_Gyro_pitch(self, msg):
        self.Gyro_pitch = msg.data

    def callback_Gyro_roll(self, msg):
        self.Gyro_roll = msg.data

    def callback_Latitude(self, msg):
        self.latitude = msg.data

    def callback_Longitude(self, msg):
        self.longitude = msg.data
        

    def timer_callback_odom(self):
        '''
        Compute the linear and angular velocity of the robot using the differential-drive robot kinematics
        Perform Euler integration to find the position x and y of the robot
        '''

        self.current_time = self.get_clock().now().nanoseconds/1e9
        
        # dt = self.current_time - self.last_time # DeltaT
        dt = 0.1
        vl = (self.blspeed + self.flspeed)/2.0  #Average Left-wheels speed
        vr = (self.brspeed + self.frspeed)/2.0  # ... Your code here. Average right-wheels speed
        
        v = (vr + vl) / 2.0     # ... Linear velocity of the robot
        #w = (vr - vl) / self.l_wheels    # ... Angular velocity of the robot
        w = self.Gyro_yaw
        # d_theta = w * dt      # Change in orientation
        # change_x = v * math.cos(self.theta) * dt
        # change_y = v * math.sin(self.theta) * dt

        self.x += v * math.cos(self.theta) * dt    # ...Position
        self.y += v * math.sin(self.theta) * dt # ...Position
        self.theta += w * dt # ...Heading angle
        # self.theta += math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize theta between -pi and pi

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
        odom.pose.pose.position.x = self.x # x_coord of robot current pos in odom frame
        odom.pose.pose.position.y = self.y # y_coord of robot current pos in odom frame
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
        t.transform.translation.z = pos[2]     # 2D

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
