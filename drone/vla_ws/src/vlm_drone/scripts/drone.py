#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

import numpy as np
import requests
import time
import math

SERVER_URL = "http://192.168.50.2:5000/predict_action"

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.rate = rospy.Rate(30)

        self.is_initiate = False
        
        self.frame = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/t265/fisheye1/image_raw', Image, self.image_callback)

        self.instruction = "Fly through the gates and avoid obstacles"  
        self.drone_actions = [0, 0, 0, 0] # vx, vy, vz, dyaw
        self.once = True

        self.counter = 0
        self.c_rate = 20

        self.vel_k_z = 0.5
        self.vel_l_xy = 1.875
        self.yaw_k = 1.5

        self.results = []
        self.yaw_drone = 0
        self.connected = False
        self.armed = False

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.wait_for_service('/mavros/cmd/land')

        rospy.Subscriber("/vicon/drone_vla/drone_vla", TransformStamped, self.orient_callback)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.orient_callback_vio)
        rospy.Subscriber("/mavros/state", State, self.callback_state, queue_size=10)

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.vel_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.vla_image_pub = rospy.Publisher('/vla_image', Image, queue_size=10)


    def prepare_vel(self, vel):
        vx_r = self.vel_l_xy * (vel[0] * math.cos(self.yaw_drone) - vel[1] * math.sin(self.yaw_drone))
        vy_r = self.vel_l_xy * (vel[0] * math.sin(self.yaw_drone) + vel[1] * math.cos(self.yaw_drone))
        vz_r = self.vel_k_z * vel[2]
        yaw_r = self.yaw_k * vel[3] 
        return [vx_r, vy_r, vz_r, yaw_r]
    
    def callback_state(self, state):
        self.connected = state.connected
        self.armed = state.armed
    
    def orient_callback_vio(self, msg):
        def euler_from_quaternion(x, y, z, w):
            """Convert a quaternion into euler angles (roll, pitch, yaw)

            Args:
                x (_type_): _description_
                y (_type_): _description_
                z (_type_): _description_
                w (_type_): _description_

            Returns:
                roll_x, pitch_y, yaw_z: is rotation around x, y, z in radians (counterclockwise)
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
        
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        r, p, y = euler_from_quaternion(x, y, z, w)
        self.yaw_drone = y

    def orient_callback(self, msg):
        x = msg.transform.rotation.x
        y = msg.transform.rotation.y
        z = msg.transform.rotation.z
        w = msg.transform.rotation.w

        def euler_from_quaternion(x, y, z, w):        
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1) 
            # roll_x = math.atan2(t0, t1) * (180 / math.pi)
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
            # pitch_y = math.asin(t2) * (180 / math.pi)
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
            # yaw_z = math.atan2(t3, t4)* (180 / math.pi)
            return roll_x, pitch_y, yaw_z # in radians

        _, _, y = euler_from_quaternion(x, y, z, w)
        self.yaw_drone = y

    def take_off_init(self):
        self.set_mode("GUIDED")
        self.takeoff(height=0.6)
        rospy.sleep(5)
        self.is_initiate = True

    def arm(self):
        """
        Sets the drone mode to "GUIDED" and arms the drone, enabling it to complete missions and tasks.
        """

        # Changes mode to "GUIDED"
        try:
            rospy.logwarn("sending mode")
            resp = self.set_mode_client(0, 'GUIDED')
            while not resp.mode_sent:
                resp = self.set_mode_client(0, 'GUIDED')
                self.rate.sleep()
            if resp.mode_sent:
                rospy.logwarn("Mode changed")
                # self.mode = "GUIDED"
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        # Arms the drone
        try:
            rospy.logwarn("sending arm request")
            resp = self.arming_client(True)
            while not resp.success:
                resp = self.arming_client(True)
                self.rate.sleep()
            if resp.success:
                rospy.logwarn("armed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def takeoff(self, height=0.5):
        """
        The takeoff strategy for the drone.
        """

        # Arm the drone
        if not self.armed:
            self.arm()

        # Takeoff
        try:
            rospy.logwarn("sending takeoff request")
            resp = self.takeoff_client(0, 0, 0, 0, height)
            while not resp.success:
                resp = self.takeoff_client(0, 0, 0, 0, height)
                self.rate.sleep()
                
            if resp.success:
                rospy.logwarn("takeoff successful")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        rospy.logwarn("takeoff END")

    def set_mode(self, mode):
        try:
            rospy.loginfo(f"Setting mode to {mode}...")
            response = self.set_mode_client(0, mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode set to {mode} successfully")
            else:
                rospy.logerr(f"Failed to set mode to {mode}")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def image_callback(self, msg):
        self.counter += 1

        if self.counter >= self.c_rate:
            self.counter = 0
            self.once = True

        if self.once:
            self.vla_image_pub.publish(msg)

            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = np.asanyarray(self.frame)

            self.results = self.process_and_predict()
            self.once = False

            try:
                self.drone_actions = self.get_action_from_json(self.results)
                if self.is_initiate: 
                    print("Result: vx: " + str(round(self.drone_actions[0], 3)) + "  " + 
                              "vy: " + str(round(self.drone_actions[1], 3)) + "  " +
                              "vz: " + str(round(self.drone_actions[2], 3)) + "  " +
                              "d_ayw " + str(round(self.drone_actions[2], 3)))
                    rospy.loginfo("-- VEL CMD SENT --")

                    self.set_vel(self.prepare_vel(self.drone_actions))
                    
            except Exception as e:
                print(f"Error: {e}")

    def set_vel(self, vel):
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = "map"
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.twist.linear.x = vel[0]
        vel_cmd.twist.linear.y= vel[1]
        vel_cmd.twist.linear.z = vel[2]
        vel_cmd.twist.angular.z = vel[3]
        self.vel_publisher.publish(vel_cmd)

    def get_action_from_json(self, data):
        for entry in data:
            result = entry.get('result', {})
            delta_yaw = result.get('delta_yaw', None)
            velocities = result.get('velocities', {})
            vx = velocities.get('x', None)
            vy = velocities.get('y', None)
            vz = velocities.get('z', None)
        return [vx, vy, vz, delta_yaw]

    def send_image_to_server(self):
        _, image_encoded = cv2.imencode('.jpg', self.frame)
        start_time = time.time()
        response = requests.post(
            SERVER_URL,
            files={"image": ("image.jpg", image_encoded.tobytes(), "image/jpeg")},
            data={"instruction": self.instruction}
        )
        end_time = time.time()
        total_time = end_time - start_time

        if response.status_code == 200:
            print(f"Server round-trip time: {total_time:.2f} seconds")
            return response.json()
        else:
            raise RuntimeError(f"Server error: {response.text}")

    def process_and_predict(self):
        results = []

        try:
            print("Processing...")
            result = self.send_image_to_server()
            results.append({"result": result})

        except Exception as e:
            print(f"Error processing: {e}")
            results.append({"error": str(e)})
        
        return results
           

if __name__ == "__main__":
    node = ImageProcessor()
    node.take_off_init()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


