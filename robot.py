from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType
from ucl.complex import motorCmd
import numpy as np
import pandas as pd
import time


def interpolate_points(points, num_points=100):
    x_coords, y_coords = zip(*points)
    t = np.linspace(0, 1, len(points))
    t_interp = np.linspace(0, 1, num_points)
    x_interp = np.interp(t_interp, t, x_coords)
    y_interp = np.interp(t_interp, t, y_coords)
    return list(zip(x_interp, y_interp))


class Robot():
    def __init__(self, app):
        self.robot_x = 0
        self.robot_y = 0
        self.robot_yaw = 0

        self.EMERGENCY_STOP = False
        self.app = app

        self.pos_data = []
        self.velocity_data = []
        self.gyro_data = []
        self.yaw_data = []
        self.distances_data = []
        self.AtTa_data = []
        self.AtTu_data = []

        self.linear_reg = Regulator(kp=1, ki=0.1, kd=0.5)
        self.side_reg = Regulator(kp=1, ki=0.1, kd=0.5)
        self.angular_reg = Regulator(kp=1, ki=0.1, kd=0.5)

        try:
            self.conn = unitreeConnection(HIGH_WIRED_DEFAULTS) #HIGH_WIFI_DEFAULTS
            self.conn.startRecv()
            self.hcmd = highCmd()
            self.hstate = highState()
            # Send empty command to tell the dog the receive port and initialize the connectin
            cmd_bytes = self.hcmd.buildCmd(debug=False)
            self.conn.send(cmd_bytes)
            time.sleep(0.5)  # Some time to collect pakets
            data = self.conn.getData()
        except Exception as e:
            print("Connection error", e)

    def before_start(self):
        self.hcmd.mode = MotorModeHigh.FORCE_STAND
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def rotate(self, yaw_speed):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.gaitType = GaitType.TROT
        self.hcmd.velocity = [0, 0]
        self.hcmd.yawSpeed = yaw_speed
        self.hcmd.footRaiseHeight = 0.1
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def move_forward(self, linear_speed):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.gaitType = GaitType.TROT
        self.hcmd.velocity = [linear_speed, 0]
        self.hcmd.yawSpeed = 0
        self.hcmd.footRaiseHeight = 0.1
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def moving_command(self, linear_speed, yaw_speed):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.gaitType = GaitType.TROT
        self.hcmd.velocity = [linear_speed, 0]
        self.hcmd.yawSpeed = yaw_speed
        self.hcmd.footRaiseHeight = 0.1
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def move(self, waypoints):
        self.before_start()
        interpolated_trajectory = interpolate_points(waypoints, num_points=20)

        for t in interpolated_trajectory:
            if self.EMERGENCY_STOP:
                self.emergency_stop()
                break
            print(f"POINT {t}")
            while True:
                if self.EMERGENCY_STOP:
                    self.emergency_stop()
                    break
                self.get_data()

                dx = t[0] - self.robot_x
                dy = t[1] - self.robot_y
                distance = np.sqrt(dx ** 2 + dy ** 2)
                angle_to_target = np.degrees(np.arctan2(dy, dx))
                angle_to_turn = angle_to_target - self.robot_yaw


                if angle_to_turn > 180:
                    angle_to_turn -= 360
                elif angle_to_turn < -180:
                    angle_to_turn += 360

                self.AtTa_data.append(angle_to_target)
                self.AtTu_data.append(angle_to_turn)
                self.distances_data.append(distance)

                print("\n---------------------------------------------------------------------------")
                print(f"Move vector: [{dx}, {dy}], Distance: {distance}, "
                      f"Angle to target: {angle_to_target}, Angle to turn: {angle_to_turn}")

                if abs(angle_to_turn) < 10 and distance < 0.05:
                    print("Reached target point.")
                    break

                self.linear_reg.setpoint = 0.4 if distance > 0.2 else 0
                self.angular_reg.setpoint = round(angle_to_turn*2/180, 2)

                linear_speed = self.linear_reg.compute(current_value=self.velocity_data[-1][0])
                yaw_speed = self.angular_reg.compute(current_value=self.gyro_data[-1][2])

                print(f"Computed speeds - Linear: {linear_speed}, Yaw: {yaw_speed}")
                #print(f"Linear speed = {linear_speed}, yaw_speed = {yaw_speed}")
                #self.moving_command(linear_speed, yaw_speed)

                try:
                    self.rotate(yaw_speed)
                except Exception as e:
                    print("Rotate error", e)

                time.sleep(0.002)

                try:
                    self.move_forward(linear_speed)
                except Exception as e:
                    print("Move forward error", e)

                self.robot_x = self.pos_data[-1][0]
                self.robot_y = self.pos_data[-1][1]
                self.robot_yaw = np.rad2deg(self.yaw_data[-1][2])

                print(f"New position: x = {self.robot_x}, y = {self.robot_y}, yaw = {self.robot_yaw}")

                self.app.map.update_robot_position(x=self.robot_x, y=self.robot_y, angle=self.robot_yaw)

                if distance < 0.05:
                    break
                time.sleep(0.05)

        self.save_data()
        self.app.control_panel.stop_movement()

    def save_data(self):
        arr_pos = np.array(self.pos_data)
        arr_vel = np.array(self.velocity_data)
        arr_gyro = np.array(self.gyro_data)
        arr_rpy = np.array(self.yaw_data)
        distances = np.array(self.distances_data)

        df = pd.DataFrame({'pos_x': arr_pos[:,0], 'pos_y': arr_pos[:,1], 'pos_z': arr_pos[:,2],
                           'vel_x': arr_vel[:,0], 'vel_y': arr_vel[:,1], 'vel_z': arr_vel[:,2],
                           'gyro_x': arr_gyro[:,0], 'gyro_y': arr_gyro[:,1], 'gyro_z': arr_gyro[:,2],
                           'roll': arr_rpy[:,0], 'pitch': arr_rpy[:,1], 'yaw': arr_rpy[:,2],
                           'distances': distances, 'AtTa': self.AtTa_data, 'AtTu': self.AtTu_data})
        df.to_csv(r"C:\Users\manke\PycharmProjects\UnitreeRobot\data\data2.csv")

    def stand_up(self):
        time.sleep(0.2)
        self.hcmd.mode = MotorModeHigh.STAND_UP
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        time.sleep(0.2)

        self.before_start()

    def stand_down(self):

        time.sleep(0.01)
        self.hcmd.mode = MotorModeHigh.STAND_DOWN
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

        time.sleep(1)
        self.hcmd.mode = MotorModeHigh.DAMPING
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def emergency_stop(self):
        self.EMERGENCY_STOP = True
        time.sleep(0.002)
        self.hcmd.mode = MotorModeHigh.STAND_DOWN
        self.hcmd.velocity = [0, 0]  # Остановить движение вперед/назад и вбок
        self.hcmd.yawSpeed = 0       # Остановить вращение
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        time.sleep(1)
        self.hcmd.mode = MotorModeHigh.DAMPING
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        self.save_data()
        print("Emergency stop activated: robot has stopped.")

    def get_data(self):
        data = self.conn.getData()
        paket = data[-1]
        self.hstate.parseData(paket)

        self.velocity_data.append(self.hstate.velocity)
        self.pos_data.append(self.hstate.position)
        self.gyro_data.append(self.hstate.imu.gyroscope)
        self.yaw_data.append(self.hstate.imu.rpy)

        #print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        #print(f'Position: {self.hstate.position}')
        #print(f'Velocity: {self.hstate.velocity}')
        #print(f'yawSpeed: {self.hstate.yawSpeed}')
        #print(f'Quaternions: {self.hstate.imu.quaternion}')
        #print(f'Gyroscope: {self.hstate.imu.gyroscope}')
        #print(f'Accelerometer: {self.hstate.imu.accelerometer}')
        #print(f'RPY: {self.hstate.imu.rpy}')


class Regulator:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error-self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

    def reset(self):
        self.last_error = 0
        self.integral = 0