import math
import time
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class PID:
    def __init__(self, kp=2, ki=0.0, kd=0.0, SetPoint=0.0, current_time=None):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.sample_time = 0.01
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear(SetPoint)

    def clear(self, SetPoint):
        self.SetPoint = SetPoint
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.windup_guard = 10.0
        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


class MyRobot1(RCJSoccerRobot):
    def run(self):
        control_th = PID(10, 0.0, 1)
        control_dis = PID(1, 0, 0)
        control_fb = PID()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                heading = self.get_compass_heading()
                robot_pos = self.get_gps_coordinates()
                direction = utils.get_direction(ball_data["direction"])
                sonar_values = self.get_sonar_values()
                print(sonar_values)

                distance = ball_data["strength"]
                theta = ball_data["direction"][1]
                FrontBack = ball_data["direction"][0]

                control_th.update(theta)
                control_dis.update(1 / distance)
                control_fb.update(FrontBack)
                if sum(val < 35 for key, val in sonar_values.items()) == 0:
                    if FrontBack < -0.6:
                        self.left_motor.setVelocity(control_fb.output)
                        self.right_motor.setVelocity(-control_fb.output)
                    elif distance < 25:
                        v = -control_dis.output
                        w = control_th.output
                        vl = (2 * v - 0.08 * w) / (2 * 0.02)
                        vr = (2 * v + 0.08 * w) / (2 * 0.02)
                        self.left_motor.setVelocity(vl)
                        self.right_motor.setVelocity(vr)
                    elif distance > 25:
                        self.left_motor.setVelocity(10)
                        self.right_motor.setVelocity(10)
                elif sonar_values["front"] < 35:
                    self.left_motor.setVelocity(-10)
                    self.right_motor.setVelocity(-10)
                elif sonar_values["back"] < 35:
                    self.left_motor.setVelocity(10)
                    self.right_motor.setVelocity(10)
                elif sonar_values["left"] < 35:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(10)
                elif sonar_values["right"] < 35:
                    self.left_motor.setVelocity(10)
                    self.right_motor.setVelocity(0)
