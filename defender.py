import math
import utils
import time
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


class MyRobot2(RCJSoccerRobot):
    def run(self):
        control_ballang = PID(2, 0, 0)
        control_balldis = PID(1, 0, 0)
        control_dis = PID(0.3, 0, 0)
        control_ang = PID(0.5, 0, 0)
        L = 0.08
        R = 0.02
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # self.left_motor.setVelocity(0)
                    # self.right_motor.setVelocity(0)
                    continue

                heading = self.get_compass_heading()
                robot_pos = self.get_gps_coordinates()
                direction = utils.get_direction(ball_data["direction"])

                # blue goal
                xd = 0.7
                yd = 0.0
                x = robot_pos[1]
                y = robot_pos[0]
                balldis = ball_data["strength"]
                ballang = ball_data["direction"][1]
                FrontBack = ball_data["direction"][0]

                if y > 0:
                    ang = -math.pi + heading - math.atan2(yd - y, -xd + x)
                else:
                    ang = math.pi + heading - math.atan2(yd - y, -xd + x)
                dis = math.sqrt((yd - y) ** 2 + (xd - x) ** 2)
                control_dis.update(dis)
                control_ang.update(ang)
                control_ballang.update(ballang)
                control_balldis.update(1 / balldis)
                if abs(ang) > 0.1 and dis > 0.1:
                    v = -control_dis.output
                    w = control_ang.output
                    vl = (2 * v - L * w) / (2 * R)
                    vr = (2 * v + L * w) / (2 * R)
                    self.left_motor.setVelocity(-vl)
                    self.right_motor.setVelocity(-vr)

                else:
                    if balldis > 60:
                        if FrontBack < -0.6:
                            self.left_motor.setVelocity(10)
                            self.right_motor.setVelocity(-10)
                        else:
                            vb = -control_balldis.output
                            wb = control_ballang.output
                            vlb = (2 * vb - L * wb) / (2 * R)
                            vrb = (2 * vb + L * wb) / (2 * R)
                            self.left_motor.setVelocity(vlb)
                            self.right_motor.setVelocity(vrb)
                    else:
                        self.left_motor.setVelocity(-1)
                        self.right_motor.setVelocity(1)
