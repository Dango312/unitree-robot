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

    def move_forward(self, distance):
        print("Move")
        for i in range(distance):
            time.sleep(0.002)
            if i%20 == 0:
                self.get_data()

            self.hcmd.mode = MotorModeHigh.VEL_WALK
            self.hcmd.gaitType = GaitType.TROT
            self.hcmd.velocity = [0.4, 0]  #
            self.hcmd.yawSpeed = 0
            self.hcmd.bodyHeight = 0.1
            self.hcmd.footRaiseHeight = 0.1
            cmd_bytes = self.hcmd.buildCmd(debug=False)
            self.conn.send(cmd_bytes)

    def rotate(self, yaw_speed):
        print("Rotate")
        for i in range(65):
            time.sleep(0.002)
            if i%20==0:
                self.get_data()

            self.hcmd.mode = MotorModeHigh.VEL_WALK
            self.hcmd.gaitType = GaitType.TROT
            self.hcmd.velocity = [0, 0]  # -1  ~ +1
            self.hcmd.yawSpeed = yaw_speed
            self.hcmd.footRaiseHeight = 0.1

            cmd_bytes = self.hcmd.buildCmd(debug=False)
            self.conn.send(cmd_bytes)

    def move(self, trajectory):
        self.before_start()
        for t in trajectory:
            if self.EMERGENCY_STOP:
                self.emergency_stop()
                break

            dx = t[0] - self.robot_x
            dy = t[1] - self.robot_y
            distance = np.sqrt(dx**2 + dy**2)
            angle_to_target = np.degrees(np.arctan2(dy, dx))
            angle_to_turn = angle_to_target - self.robot_yaw

            if angle_to_turn > 180:
                angle_to_turn -= 360
            elif angle_to_turn < -180:
                angle_to_turn += 360

            print(f"Move vector: [{dx}, {dy}], Distance: {distance}, Angle to target: {angle_to_target}, Angle to turn: {angle_to_turn}")

            Ud = int(150 * distance)
            ys = round(angle_to_turn*2/180, 2)
            print(f"Ud = {Ud}, ys = {ys}")
            try:
                self.rotate(ys)
            except Exception as e:
                print("Rotate error", e)

            time.sleep(1)
            try:
                self.move_forward(Ud)
            except Exception as e:
                print("Move forward error", e)

            self.robot_x = t[0]
            self.robot_y = t[1]
            self.robot_yaw += angle_to_turn

            if self.robot_yaw > 180:
                self.robot_yaw -= 360
            elif self.robot_yaw < -180:
                self.robot_yaw += 360

            print(f"New position: x = {self.robot_x}, y = {self.robot_y}, yaw = {self.robot_yaw}")

            self.app.map.update_robot_position(x=self.robot_x, y=self.robot_y, angle=self.robot_yaw)
            time.sleep(1)


        arr_pos = np.array(self.pos_data)
        arr_vel = np.array(self.velocity_data)
        arr_gyro = np.array(self.gyro_data)
        arr_rpy = np.array(self.yaw_data)

        df = pd.DataFrame({'pos_x': arr_pos[:,0], 'pos_y': arr_pos[:,1], 'pos_z': arr_pos[:,2],
                           'vel_x': arr_vel[:,0], 'vel_y': arr_vel[:,1], 'vel_z': arr_vel[:,2],
                           'gyro_x': arr_gyro[:,0], 'gyro_y': arr_gyro[:,1], 'gyro_z': arr_gyro[:,2],
                           'roll': arr_rpy[:,0], 'pitch': arr_rpy[:,1], 'yaw': arr_rpy[:,2]})
        df.to_csv(r"C:\Users\manke\PycharmProjects\UnitreeRobot\data\data.csv")

    def stand_up(self):
        time.sleep(0.2)
        self.hcmd.mode = MotorModeHigh.STAND_UP
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        time.sleep(0.2)
        try:
            self.get_imu()
        except Exception as e:
            print("Something wrong with IMU", e)
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
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        time.sleep(1)
        self.hcmd.mode = MotorModeHigh.DAMPING
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)

    def get_data(self):
        data = self.conn.getData()
        paket = data[-1]
        self.hstate.parseData(paket)

        self.velocity_data.append(self.hstate.velocity)
        self.pos_data.append(self.hstate.position)
        self.gyro_data.append(self.hstate.imu.gyroscope)
        self.yaw_data.append(self.hstate.imu.rpy)

        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        print(f'Position: {self.hstate.position}')
        print(f'Velocity: {self.hstate.velocity}')
        print(f'yawSpeed: {self.hstate.yawSpeed}')
        print(f'Quaternions: {self.hstate.imu.quaternion}')
        print(f'Gyroscope: {self.hstate.imu.gyroscope}')
        print(f'Accelerometer: {self.hstate.imu.accelerometer}')
        print(f'RPY: {self.hstate.imu.rpy}')

