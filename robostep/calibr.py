#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.parameters import Color as Col
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import Font

import os


class Robot:

    def __init__(self):
        self.init_vars()
        self.init_hardware()


    # инит хардваре
    def init_hardware(self): # инициализация оборудования
        self.ev3 = EV3Brick()

        # Initialize the motors.
        self.left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)
        # self.middle_right_motor = Motor(Port.A)
        # self.middle_left_motor = Motor(Port.D)

        # Initialize the color sensor.
        self.line_sensor = ColorSensor(Port.S1)
        self.line_sensor_1 = ColorSensor(Port.S2)
        self.line_sensor_2 = ColorSensor(Port.S3)
        # self.middle_color_sensor = ColorSensor(Port.S4)
        
        # try:
        #     self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        # except: ...

        # Initialize the drive base.
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=62.4, axle_track=195)
        self.robot.settings(straight_speed=self.default_speed)

        self.touch_sensor_1, self.touch_sensor_2 = None, None

        try:
            self.touch_sensor_1 = TouchSensor(Port.S4)
            # self.touch_sensor_2 = TouchSensor(Port.S2)

        except:
            pass


    # инитим переменные
    def init_vars(self): # инициализация переменных

        self.color_list = [0]*10
        self.run_list = list()
        self.cube_list = list()
        self.d = 62.4

        self.default_speed = 350 # инициализация дефолтной скорости

        self.active_cube = 0  # 0 - left 1 - right
        self.last_error = 0 # инициализация last_error
        self.error = 0 # инициализация error
        self.I = 0 # инициализация I

        self.gray = 35 # инициализация среднего значения между белым и черным

        self.counter = 1 # инициализация counter

        self.colors_int = { # инициализация цветов
            Col.BLACK: 1,
            Col.BLUE: 2,
            Col.GREEN: 3,
            Col.YELLOW: 4,
            Col.RED: 5,
            Col.WHITE: 6,
            Col.BROWN: 7,
            Col.ORANGE: 8,
            Col.PURPLE: 9,
        }

    def wait_pressed(self):
        while not self.touch_sensor_1.pressed(): pass
        wait(300)


    def run_line(self, speed=350, kp=2, kd=20, ki=0.02):
        self.error = self.line_sensor_1.reflection() - self.line_sensor_2.reflection()
        P = kp * self.error
        D = kd * (self.error - self.last_error)
        self.I += ki * self.error
        U = P + D + self.I
        self.left_motor.run(speed + U)
        self.right_motor.run(speed - U)
        self.last_error = self.error
        wait(10)
# функция для определения моды(доложно считывать через count кол-во элементов в self.color_list)
    def count_mode(self):

        max_counter_list=0
        set_mode = set(self.color_list)


    def check_col(self, dist=90, speed = 200):
        temp_color_list = []
        

        for _ in range(3):
            while self.robot.distance() <= dist:
                self.run_line(speed=speed)
                temp_color_list.append(self.line_sensor.rgb())
                wait(10)
            temp_color_list.append("|")
            self.robot.reset()
            self.right_motor.stop()
            self.left_motor.stop()
            self.wait_pressed() 


            # temp_color_list.clear()
        print(temp_color_list)
        return temp_color_list



    def main(self) -> None:
        self.wait_pressed() 
        # self.check_col()
        f = open('colors.txt','w') # открытие в режиме записи
        vars = ' '.join(self.check_col())
        print(vars, file=f)
        f.close()  # закрытие файла
    

r = Robot()
r.main()

# f = open('xyz.txt','w')  # открытие в режиме записи
# f.write('Hello \n World')  # запись Hello World в файл
# f.close()  # закрытие файла



        # print(temp_color_list)
            # Далее по списку кортежей определяем модный и записываем его в self.color_list
            # self.color_list.append(mode(list(map(str, temp_color_list))))