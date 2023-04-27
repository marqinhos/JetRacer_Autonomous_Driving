#!/usr/bin/env python3

import rospy
from jetracer_speedway_msgs.msg import Points
from jetracer_speedway_msgs.msg import Velocities
import threading as th
import numpy as np
import signal
import yaml


class Brain(th.Thread):


    def __init__(self) -> "Brain":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### YAML ###########################
        # Load config.yaml
        with open(rospy.get_param("/jetracer_speedway_bringup/config_file"), 'r') as f:
            config = yaml.safe_load(f)

        ########################### ROS ###########################
        ## Constants
        self.name_sub = config["navigation"]["pub_name_img"]
        self.name_pub = config["control"]["pub_name"]
        self.name_ros_node = config["control"]["node_name"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.sub_point = rospy.Subscriber(self.name_sub, Points, self.__callback_point)
        self.pub_vels = rospy.Publisher(self.name_pub, Velocities, queue_size=1)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)

        ########################### Running ###########################
        self.running = True   

        ########################### Point ###########################
        self.points = None


    def run(self) -> None:
        while self.running:
            ## Wait for a frame
            if self.points is None:
                self.rospyRate.sleep()
                continue
            
            ## Pass desired point to angle
            angle = self.__convert_point_2_vel(self.points)
            ## Publish desired point
            self.publish_vels(angle)

            rospy.loginfo(f"Angle: {angle}")
            self.rospyRate.sleep()


    def __callback_point(self, msg: Points) -> None:
        self.points = Points


    def publish_vels(self, angle_deg: float) -> None:
            """Void to publish angular vel and linear vel in Twist format. The topic to publish is "jetracer_vels"

            Args:
                angle_deg (float): Angle
            """
            CONSTANT_VEL = 0.6
            vel = 0.05
            
            #if not same:
            if angle_deg > 0:
                vel = 4.4*(1/abs(angle_deg))

            else:
                vel = CONSTANT_VEL
            
            # Break velocity
            vel = vel if vel <= CONSTANT_VEL else CONSTANT_VEL
            # Create Twist message
            vel_msg = Velocities()
            vel_msg.angular = angle_deg
            vel_msg.linear = vel
            
            # Publish message in topic vels_jetracer
            self.pub_vels.publish(vel_msg)


    def __convert_point_2_vel(self, points: Points) -> float:
            """Function to extract angle from a goal point respect real point.
                To calculate angle use trigonometry.
                    - Calculate horizontal side
                    - Calculate vertical side
                    - Calculate angle with arctang
                    - To select the turn sign i do the next:
                        If go to right, that is, the desired point is to the right of the real point the sign is negative.
                        In the other case is positive.

            Args:
                goal_point (Point): Desired Point calculate below

            Returns:
                float: Return angle in degrees
            """

            ## Calculate sides
            horizontal_side = points.xr - points.x
            vertical_side = points.yr - points.y
            ## Take sign of turn
            sign = 1 if horizontal_side >= 0 else -1
            ## Calculate theta
            theta_rad = np.arctan2(abs(horizontal_side), vertical_side)
            theta_degree = theta_rad * 180.0 / np.pi
            ## Bounded theta for max value to jetracer
            theta_deg_bounded = theta_degree * 32.2 / 80

            return theta_deg_bounded*sign


def signal_handler(signal, frame):
    rospy.logwarn("Ctrl+C detected, stopping threads...")
    driver_control.running = False


if __name__ == '__main__':
    try:
        ## Call JetRacer and Brain classes
        driver_control = Brain()
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Upload threads
        list_threads = [driver_control]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass