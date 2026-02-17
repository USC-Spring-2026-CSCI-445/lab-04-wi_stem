#!/usr/bin/env python3
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0.0
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        u = self.kP * err
        u = max(self.u_min, min(u, self.u_max))
        self.t_prev = t
        return u
        ######### Your code ends here #########


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.kD = kD
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0.0
        self.err_prev = 0.0
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        derr = (err - self.err_prev) / dt
        u = self.kP * err + self.kD * derr
        u = max(self.u_min, min(u, self.u_max))
        self.err_prev = err
        self.t_prev = t
        return u
        ######### Your code ends here #########


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using a sonar sensor.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.cliff_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_state_callback, queue_size=1)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define PD controller for wall following here
        ######### Your code starts here #########
        # Reduced gains for real hardware (less noisy than sim)
        kP = 1.2
        kD = 0.8
        u_min = -1.5
        u_max = 1.5

        self.controller = PDController(kP, kD, u_min, u_max)
        ######### Your code ends here #########

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None

    def sensor_state_callback(self, state: SensorState):
        raw = state.cliff
        ######### Your code starts here #########
        # conversion from raw sensor values to distance. Use equation from Lab 2
        raw_val = float(raw)
        raw_val = max(raw_val, 1.0)
        distance = 1597 * pow(raw_val, -1.522)
        ######### Your code ends here #########
        # print(f"raw: {raw}\tdistance: {distance}")
        self.ir_distance = distance

    def control_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.ir_distance is None:
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            # --- CHANGE 1: Flip the Error calculation ---
            # Now: Positive = Too Far, Negative = Too Close
            err = self.ir_distance - self.desired_distance
            
            u = self.controller.control(err, time())
            if self.ir_distance < self.desired_distance:
                u = max(u, -0.6)
                ctrl_msg.linear.x = 0.08

            elif self.ir_distance > self.desired_distance + 0.25:
                u = min(u, 0.9)
                ctrl_msg.linear.x = 0.12

            else:
                ctrl_msg.linear.x = 0.16
            # slower forward speed for stability on hardware
            
            ctrl_msg.angular.z = u

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {self.desired_distance}\tu: {round(u, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.4
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
