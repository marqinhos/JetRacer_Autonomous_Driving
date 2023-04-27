from adafruit_servokit import ServoKit
import numpy as np
import rospy
import threading as th
import yaml

class JetRacer(th.Thread):

    def __init__(self) -> None:
        ########################### Threads ###########################
        th.Thread.__init__(self)
        
        ########################### YAML ###########################
        # Load config.yaml
        with open(rospy.get_param("/jetracer_speedway_bringup/config_file"), 'r') as f:
            config = yaml.safe_load(f)

        ##################### JetRacer Constants ######################
        self.__i2c_address = config["driver"]["i2c_address"]
        self.__steering_channel = config["driver"]["steering_channel"]
        self.__throttle_channel = config["driver"]["throttle_channel"]

        #################### JetRacer Connection ######################
        self.kit = ServoKit(channels=16, address=self.__i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.__steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.__throttle_channel]

        ######################### Running #############################
        self.running = True
        self.run_stop_emergency = False

        ##################### JetRacer Constants ######################
        self.__max_vel = config["driver"]["max_speed"]
        self.__min_vel = config["driver"]["min_speed"]
        self.__max_angle_turn = config["driver"]["max_angle"]
        self.__min_angle_turn = config["driver"]["min_angle"]

        ##################### JetRacer Vels ###########################
        self.current_value_vel = 0
        self.current_value_turn = 0

        self.circle_center = 0

        ########################### RATE ##############################
        self.rate = 0


    def set_rate(self, rate: int):
        """Function to set the rate frequency

        Args:
            rate (int): rate frequency
        """
        self.rate = rate


    def run(self):
        """Process to run the program. Inside it only set the velocity and turn to adafruit motors.
            If there are some problems, they show into logerror ROS. 
        """
        while self.running:
            ####################################################
            ## Maybe add pid controller to reduce delta error ##
            ####################################################
            if not self.run_stop_emergency:
                try:
                    self.steering_motor.throttle = self.current_value_turn
                    self.throttle_motor.throttle = self.current_value_vel
                    self.rate.sleep()
                    #self.rate.sleep()
                except Exception as err:
                    rospy.logerr(err.args)
        
        rospy.loginfo("JetRacer is closed")


    def stop(self):
        """Function to STOP JetRacer
        """
        self.current_value_turn = 0
        self.current_value_vel = 0
        self.steering_motor.throttle = self.current_value_turn
        self.throttle_motor.throttle = self.current_value_vel
        self.running = False


    def stop_emergency(self):
        """Function to STOP emergency
        """
        self.current_value_turn = 0
        self.current_value_vel = 0
        self.steering_motor.throttle = self.current_value_turn
        self.throttle_motor.throttle = self.current_value_vel
        self.run_stop_emergency = True


    def set_angle(self, angle: float) -> None:
        """Function to update the turn's value to new value 

        Args:
            new_turn (float): New value of turn. As default 0.0 to reset.

        """
        try:
            value_turn = angle / self.__max_angle_turn
            if abs(value_turn) > 1: 
                rospy.logwarn("Module of angle must be under 32.2º")
                value_turn = 1 if value_turn > 1 else -1

            self.current_value_turn = value_turn
        except Exception as err:
            rospy.logerr(err.args)
            raise err.args
        

    def set_vel(self, vel: float) -> None:
        """Function to update velocity value, to new value

        Args:
            new_vel (float): New value of velocity you want. As default 0.0 to reset.

        """
        try:
            value_vel = vel / self.__max_vel
            if abs(value_vel) > 1: 
                rospy.logwarn("Module of vel must be under 0.8 m/s")
                value_vel = 1 if value_vel > 1 else -1

            self.current_value_vel = value_vel
        except Exception as err:
            rospy.logerr(err.args)
            raise err.args
    

    def get_current_vel(self) -> float:
        """Function to get the current value of velocity

        Returns:
            float: Value of velocity
        """
        return self.current_value_vel * self.__max_vel
        

    def get_current_angle(self) -> float:
        """Function to get the current value of turn

        Returns:
            float: Value of turn
        """
        return self.current_value_turn * self.__max_angle_turn
