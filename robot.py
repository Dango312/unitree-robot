import math

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

from fuzzylogic.classes import Domain, Rule
from fuzzylogic.functions import S, R
from fuzzylogic.functions import triangular

V = Domain("V", 0, 0.4, res=0.01)
V.o = S(0, 0.01)
V.vl = triangular(0.05, 0.15, c=0.1) #very low
V.l = triangular(0.1, 0.35, c=0.20)
V.h = R(0.3, 0.4) #high

yawSpeed = Domain("yawSpeed", -2, 2, res=0.01)
yawSpeed.vvfn = S(-2, -1.7) #very very fast negative
yawSpeed.vfn = triangular(-1.8, -0.8)
yawSpeed.fn = triangular(-0.8, -0.4) #fast negative
yawSpeed.sn = triangular(-0.4, -0.2) #slow negative
yawSpeed.vsn = triangular(-0.2, -0.1)
yawSpeed.o = triangular(-0.02, 0.02)
yawSpeed.vsp = triangular(0.1, 0.2)
yawSpeed.sp = triangular(0.2, 0.4)
yawSpeed.fp = triangular(0.4, 0.8)
yawSpeed.vfp = triangular(0.8, 1.8)
yawSpeed.vvfp = R(1.7, 2)

distance = Domain("Distance", 0, 2, res=0.01) #ошибка
distance.o = triangular(0, 0.05, c=0.015)
distance.n = triangular(0.04, 0.4, c=0.25)
distance.f = triangular(0.35, 0.6, c=0.5)
distance.vf = R(0.6, 1)

Ephi = Domain("Ephi", -180, 180) # ошибка по углу
Ephi.tvfn = S(-180, -140) #too very far negative
Ephi.vvfn = triangular(-150, -120, c=-135) #very very far negative
Ephi.vfn = triangular(-130, -90, c=-115)
Ephi.fn = triangular(-95, -50, c=-75)
Ephi.nn = triangular(-60, -40, c=-50) #near negative
Ephi.vnn = triangular(-45, -17, c=-30)
Ephi.tnn = triangular(-20, -7, c=-14) #too near negative
Ephi.o = triangular(-7, 7, c=0) #zero
Ephi.tnp = triangular(7, 20, c=14)
Ephi.vnp = triangular(17, 45, c=30) #very near posiive
Ephi.np = triangular(40, 60, c=50)
Ephi.fp = triangular(50, 95, c=75) #far positive
Ephi.vfp = triangular(90, 130, c=115)
Ephi.vvfp = triangular(120, 150, c=135)
Ephi.tvfp = R(140, 180)


velocity_rules = Rule({
    (distance.o, Ephi.o): V.o,
    (distance.n, Ephi.o): V.l,
    (distance.f, Ephi.o): V.h,
    (distance.vf, Ephi.o): V.h,
    # positive angles
})

yawSpeed_rules = Rule({
    # negative angles
    (distance.o, Ephi.tvfn): yawSpeed.vvfn,
    (distance.n, Ephi.tvfn): yawSpeed.vvfn,
    (distance.f, Ephi.tvfn): yawSpeed.vvfn,
    (distance.vf, Ephi.tvfn): yawSpeed.vvfn,

    (distance.o, Ephi.vvfn): yawSpeed.sn,
    (distance.n, Ephi.vvfn): yawSpeed.sn,
    (distance.f, Ephi.vvfn): yawSpeed.vvfn,
    (distance.vf, Ephi.vvfn): yawSpeed.vvfn,

    (distance.o, Ephi.vfn): yawSpeed.sn,
    (distance.n, Ephi.vfn): yawSpeed.sn,
    (distance.f, Ephi.vfn): yawSpeed.vvfn,
    (distance.vf, Ephi.vfn): yawSpeed.vvfn,

    (distance.o, Ephi.fn): yawSpeed.sn,
    (distance.n, Ephi.fn): yawSpeed.vfn,
    (distance.f, Ephi.fn): yawSpeed.vvfn,
    (distance.vf, Ephi.fn): yawSpeed.vvfn,

    (distance.o, Ephi.nn): yawSpeed.vsn,
    (distance.n, Ephi.nn): yawSpeed.vfn,
    (distance.f, Ephi.nn): yawSpeed.vvfn,
    (distance.vf, Ephi.nn): yawSpeed.vvfn,

    (distance.o, Ephi.vnn): yawSpeed.sn,
    (distance.n, Ephi.vnn): yawSpeed.sn,
    (distance.f, Ephi.vnn): yawSpeed.sn,
    (distance.vf, Ephi.vnn): yawSpeed.sn,

    (distance.o, Ephi.tnn): yawSpeed.vsn,
    (distance.n, Ephi.tnn): yawSpeed.sn,
    (distance.f, Ephi.tnn): yawSpeed.vfn,
    (distance.vf, Ephi.tnn): yawSpeed.vsn,

    (distance.o, Ephi.o): yawSpeed.o,
    (distance.n, Ephi.o): yawSpeed.o,
    (distance.f, Ephi.o): yawSpeed.o,
    (distance.vf, Ephi.o): yawSpeed.o,
    # positive angles
    (distance.o, Ephi.tnp): yawSpeed.vsp,
    (distance.n, Ephi.tnp): yawSpeed.vsp,
    (distance.f, Ephi.tnp): yawSpeed.vsp,
    (distance.vf, Ephi.tnp): yawSpeed.sp,

    (distance.o, Ephi.vnp): yawSpeed.vsp,
    (distance.n, Ephi.vnp): yawSpeed.vsp,
    (distance.f, Ephi.vnp): yawSpeed.sp,
    (distance.vf, Ephi.vnp): yawSpeed.vfp,

    (distance.o, Ephi.np): yawSpeed.vsp,
    (distance.n, Ephi.np): yawSpeed.vsp,
    (distance.f, Ephi.np): yawSpeed.vvfp,
    (distance.vf, Ephi.np): yawSpeed.vvfp,

    (distance.o, Ephi.fp): yawSpeed.sp,
    (distance.n, Ephi.fp): yawSpeed.sp,
    (distance.f, Ephi.fp): yawSpeed.vfp,
    (distance.vf, Ephi.fp): yawSpeed.vvfp,

    (distance.o, Ephi.vfp): yawSpeed.fp,
    (distance.n, Ephi.vfp): yawSpeed.fp,
    (distance.f, Ephi.vfp): yawSpeed.vfp,
    (distance.vf, Ephi.vfp): yawSpeed.vvfp,

    (distance.o, Ephi.vvfp): yawSpeed.fp,
    (distance.n, Ephi.vvfp): yawSpeed.vfp,
    (distance.f, Ephi.vvfp): yawSpeed.vfp,
    (distance.vf, Ephi.vvfp): yawSpeed.vvfp,

    (distance.o, Ephi.tvfp): yawSpeed.vfp,
    (distance.n, Ephi.tvfp): yawSpeed.vvfp,
    (distance.f, Ephi.tvfp): yawSpeed.vvfp,
    (distance.vf, Ephi.tvfp): yawSpeed.vvfp
})


def predict(X, F, U, P, Q):
    newX = F @ X + U
    newP = F @ P @ F.T + Q
    return newX, newP


def sense(Z, X, H, P, R):
    K = P @ H.T @ np.linalg.inv((H @ P @ H.T) + R)
    newX = X + K @ (Z - H @ X)
    newP = (np.eye(X.shape[0]) - K @ H) @ P
    return newX, newP


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

        self.linear_reg = Regulator(kp=2, ki=0, kd=0.1)
        self.side_reg = Regulator(kp=1, ki=0.1, kd=0.5)
        self.angular_reg = Regulator(kp=2, ki=0, kd=0.1)

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
        self.save_data()

    def move_forward(self, v, vs): #v-linear; vs - side
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.gaitType = GaitType.TROT
        self.hcmd.velocity = [v, vs]
        self.hcmd.yawSpeed = 0
        self.hcmd.footRaiseHeight = 0.1
        cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        self.save_data()

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
        interpolated_trajectory = interpolate_points(waypoints, num_points=10)

        f = np.array([
            [lambda x, vx: x + 0.025 * vx],
            [lambda y, vy: y + 0.025 * vy],
            [lambda vx, vy, alpha: math.sqrt(vx ** 2 + vy ** 2) * np.cos(np.deg2rad(alpha))],
            [lambda vx, vy, alpha: math.sqrt(vx ** 2 + vy ** 2) * np.sin(np.deg2rad(alpha))],
            [lambda alpha: alpha]
        ])

        dfvxdvx = lambda vx, vy, alpha: vx * np.cos(alpha) / math.sqrt(vx ** 2 + vy ** 2) if math.sqrt(
            vx ** 2 + vy ** 2) != 0 else 0
        dfvxdvy = lambda vx, vy, alpha: vy * np.cos(alpha) / math.sqrt(vx ** 2 + vy ** 2) if math.sqrt(
            vx ** 2 + vy ** 2) != 0 else 0
        dfvxdalpha = lambda vx, vy, alpha: -np.sin(alpha) * math.sqrt(vx ** 2 + vy ** 2)
        dfvydvx = lambda vx, vy, alpha: vx * np.sin(alpha) / math.sqrt(vx ** 2 + vy ** 2) if math.sqrt(
            vx ** 2 + vy ** 2) != 0 else 0
        dfvydvy = lambda vx, vy, alpha: vy * np.sin(alpha) / math.sqrt(vx ** 2 + vy ** 2) if math.sqrt(
            vx ** 2 + vy ** 2) != 0 else 0
        dfvydalpha = lambda vx, vy, alpha: np.cos(alpha) * math.sqrt(vx ** 2 + vy ** 2)
        F = np.array([
            [1, 0, 0.025, 0, 0],
            [0, 1, 0, 0.025, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1]
        ])

        def moveEKF(X, U, P, f, F, Q):
            F[2, 2] = dfvxdvx(X[2, 0], X[3, 0], X[4, 0])
            F[2, 3] = dfvxdvy(X[2, 0], X[3, 0], X[4, 0])
            F[2, 4] = dfvxdalpha(X[2, 0], X[3, 0], X[4, 0])
            F[3, 2] = dfvydvx(X[2, 0], X[3, 0], X[4, 0])
            F[3, 3] = dfvydvy(X[2, 0], X[3, 0], X[4, 0])
            F[3, 4] = dfvydalpha(X[2, 0], X[3, 0], X[4, 0])
            newX = np.array([
                [f[0, 0](X[0, 0], X[2, 0])],
                [f[1, 0](X[1, 0], X[3, 0])],
                [f[2, 0](X[2, 0], X[3, 0], X[4, 0])],
                [f[3, 0](X[2, 0], X[3, 0], X[4, 0])],
                [f[4, 0](X[4, 0])]
            ])
            newP = F @ P @ F.T + Q
            return newX, newP

        HGPS = np.eye(5)

        Q = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0.03, 0, 0],
            [0, 0, 0, 0.03, 0],
            [0, 0, 0, 0, 5]]
        )

        R2 = np.array([
            [0.05, 0, 0, 0, 0],
            [0, 0.05, 0, 0, 0],
            [0, 0, 0.02, 0, 0],
            [0, 0, 0, 0.02, 0],
            [0, 0, 0, 0, 2]
        ])

        X = np.zeros((5, 1))
        P = np.array([
            [3, 0, 0, 0, 0],
            [0, 3, 0, 0, 0],
            [0, 0, 0.1, 0, 0],
            [0, 0, 0, 0.1, 0],
            [0, 0, 0, 0, 5]
        ])

        answer = []
        p0 = [0, 0]
        for t in waypoints:
            if self.EMERGENCY_STOP:
                self.emergency_stop()
                break
            print(f"NEXT POINT {t}")

            while True:
                try:
                    self.get_data()
                except:
                    print('get_data error')

                dx = t[0] - self.robot_x
                dy = t[1] - self.robot_y
                dist = np.sqrt(dx ** 2 + dy ** 2)
                angle_to_target = np.degrees(np.arctan2(dy, dx))

                d = np.array([t[0] - p0[0], t[1] - p0[1]])
                r = np.array([np.cos(self.robot_yaw), np.sin(self.robot_yaw)])
                d_norm = np.linalg.norm(d)
                r_norm = np.linalg.norm(r)

                angle_to_turn = angle_to_target - np.rad2deg(self.robot_yaw)

                if angle_to_turn > 180:
                    angle_to_turn -= 360
                elif angle_to_turn < -180:
                    angle_to_turn += 360

                if -7 < angle_to_turn < 7:
                    angle_to_turn = 0

                theta = np.degrees(np.arccos(np.dot(d, r) / (d_norm * r_norm)))
                print(d, r, theta)
                if abs(theta) <= 5:
                    angle_to_target = 0
                    angle_to_turn = 0

                self.AtTa_data.append(angle_to_target)
                self.AtTu_data.append(angle_to_turn)
                self.distances_data.append(dist)

                print("\n---------------------------------------------------------------------------")
                print(f"Move vector: [{dx}, {dy}], Distance: {dist}, "
                      f"Angle to target: {angle_to_target}, Angle to turn: {angle_to_turn}")
                """
                moving_time = int(150 * distance)
                if angle_to_turn>10 or angle_to_turn<-10:
                    yaw_speed = round(angle_to_turn*2/180, 2)
                else:
                    yaw_speed=0
                print(f"Ud={moving_time}, Ys={yaw_speed}")"""

                try:
                    values = {distance: float(dist), Ephi: float(angle_to_turn)}
                    v = velocity_rules(values) if velocity_rules(values) else 0
                    ys = yawSpeed_rules(values)
                    print(f"V = {v}, ys={ys}")
                    if -0.07 < ys < 0.07:
                        ys = 0
                    if v < 0.05:
                        v = 0
                    print(f"V = {v}, ys={ys}")
                except Exception as e:
                    print(e)

                if ys != 0:
                    try:
                        self.rotate(ys)
                    except Exception as e:
                        print("Rotate error", e)
                        break
                time.sleep(0.001)
                try:
                    self.move_forward(v, 0)
                except Exception as e:
                    print("Move forward error", e)
                    break
                time.sleep(0.001)

                print(f"2) Gyro_speed = {self.gyro_data[-1][0]}, velocity = {self.velocity_data[-1][0]}, {self.velocity_data[-1][1]}\n"
                      f"Robot measurements: x = {self.pos_data[-1][0]}, y = {self.pos_data[-1][1]}, yaw = {self.yaw_data[-1][2]}")

                X, P = moveEKF(X, np.zeros((5,1)), P, f, F, Q)

                X, P = sense(np.array([
                    [self.pos_data[-1][0]],
                    [self.pos_data[-1][1]],
                    [self.velocity_data[-1][0]*np.cos(self.robot_yaw)],
                    [self.velocity_data[-1][0]*np.sin(self.robot_yaw)],
                    [self.yaw_data[-1][2]]
                ]), X, HGPS, P, R2)
                answer.append(X)

                self.robot_yaw = X[4][0]
                self.robot_x = X[0][0]
                self.robot_y = X[1][0]
                print(f"VelX = {X[2][0]}, VelY = {X[3][0]}")
                print(f"Kalman New position: x = {self.robot_x}, y = {self.robot_y},\n"
                      f"yaw_deg = {np.rad2deg(self.robot_yaw)}; yaw_rad = {self.robot_yaw}")

                if dist <= 0.07:
                    break

            self.app.map.update_robot_position(x=self.robot_x, y=self.robot_y, angle=self.robot_yaw)
            p0 = t
            time.sleep(0.05)



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