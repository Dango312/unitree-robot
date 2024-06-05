import tkinter as tk
import ttkbootstrap as ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from robot import Robot


class App(ttk.Window):
    def __init__(self, title, size):
        super().__init__()
        self.title(title)
        self.geometry(f'{size[0]}x{size[1]}')

        self.columnconfigure([0,1,2,3], weight=1)
        self.rowconfigure(0, weight=2)
        self.rowconfigure(1, weight=1)

        self.robot = Robot(self)
        self.map = Map(self, self.robot)
        self.control_panel = ControlPanel(self, self.map, self.robot)

        self.mainloop()


class Map(ttk.Frame):
    def __init__(self, parent, robot):
        super().__init__(parent)

        self.grid(row=0, column=2, columnspan=2, sticky='ns')
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self.robot = robot
        self.coords = {'x':0, "y":0}
        self.points = []
        self.robot_position = [0, 0]
        self.robot_angle = 0

        fig, self.ax = plt.subplots(figsize=(4, 4))
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xticks(np.arange(-5, 6, 1))
        self.ax.set_yticks(np.arange(-5, 6, 1))
        self.ax.grid()

        map_label = ttk.LabelFrame(self, text='Map')
        map_label.grid(row=0, column=0, sticky='nsew')

        self.map = FigureCanvasTkAgg(fig, master=self)
        self.map.get_tk_widget().grid(row=0, column=0, sticky='ew')
        self.map.draw()
        self.update_robot_position(x=0, y=0, angle=0)

        self.map.mpl_connect('button_press_event', self.click)

    def click(self, event):
        x = event.xdata
        y = event.ydata
        if x is None or y is None:
            return

        point_name = f'p{len(self.points) + 1}'
        self.points.append((x, y, point_name))
        self.ax.plot(x, y, 'bo')
        self.ax.annotate(point_name, (x, y), textcoords="offset points", xytext=(5, 5), ha='center')
        self.map.draw()

    def delete_last_point(self):
        if self.points:
            self.points.pop()
            self.robot_position = [self.robot.robot_x, self.robot.robot_y]
            self.robot_angle = self.robot.robot_yaw
            self.update_map()

    def update_robot_position(self, x=None, y=None, angle=None):
        if x is not None:
            self.robot_position[0] = x
        if y is not None:
            self.robot_position[1] = y
        if angle is not None:
            self.robot_angle = angle
        self.update_map()

    def update_map(self):
        self.ax.clear()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xticks(np.arange(-5, 6, 1))
        self.ax.set_yticks(np.arange(-5, 6, 1))
        self.ax.grid()
        for i, point in enumerate(self.points):
            x, y, name = point
            self.ax.plot(x, y, 'bo')
            self.ax.annotate(name, (x, y), textcoords="offset points", xytext=(5, 5), ha='center')

        robot_x, robot_y = self.robot_position
        robot_angle_rad = np.radians(self.robot_angle)
        arrow_length = 0.5

        arrow_dx = arrow_length * np.cos(robot_angle_rad)
        arrow_dy = arrow_length * np.sin(robot_angle_rad)

        self.ax.arrow(robot_x, robot_y, arrow_dx, arrow_dy, head_width=0.2, head_length=0.2, fc='red', ec='red')

        self.map.draw()

    def save_trajectory(self):
        trajectory = []
        for point in self.points:
            x, y, _ = point

            trajectory.append([x, y])

        print("Trajectory saved:", trajectory)
        return trajectory


class ControlPanel(ttk.Frame):
    def __init__(self, parent, map_widget, robot):
        super().__init__(parent)

        self.map_widget = map_widget
        self.robot = robot

        self.grid(row=0, column=0, sticky='nsew')
        self.rowconfigure([0,1,2,3, 4], weight=1)
        self.columnconfigure(0, weight=1)

        self.start_btn = ttk.Button(self, text="Start",
                                    command=self.start_movement, bootstyle=ttk.PRIMARY)
        self.start_btn.grid(row=0, column=0, sticky='ew')

        self.delete_btn = ttk.Button(self, text='Delete',
                                     command=self.delete_point, bootstyle=ttk.PRIMARY)
        self.delete_btn.grid(row=1, column=0, sticky='ew')

        self.standUp_btn = ttk.Button(self, text='Stand up',
                                      command=self.robot.stand_up, bootstyle=ttk.PRIMARY)
        self.standUp_btn.grid(row=2, column=0, sticky='ew')

        self.standDown_btn = ttk.Button(self, text='Stand down',
                                        command=self.robot.stand_down, bootstyle=ttk.PRIMARY)
        self.standDown_btn.grid(row=3, column=0, sticky='ew')

        self.standDown_btn = ttk.Button(self, text='Emergency stop',
                                        command=self.robot.emergency_stop, bootstyle=ttk.PRIMARY)
        self.standDown_btn.grid(row=4, column=0, sticky='ew')

    def start_movement(self):
        trajectory = self.map_widget.save_trajectory()
        try:
            self.robot.move(trajectory)
        except Exception as e:
            print("Something wrong with movement", e)

    def delete_point(self):
        self.map_widget.delete_last_point()


if __name__ == '__main__':
    app = App('Robot control', [1280, 720])
