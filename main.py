#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.parameters import Color as Col
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import Font
import math
from math import floor


# Создаем класс Robot(дальнейший код будем писать в нём)
class Robot:

    # Инициализируем функции init
    def __init__(self):
        self.init_vars()
        self.init_hardware()


    # Инициализация оборудования
    def init_hardware(self): 
        self.ev3 = EV3Brick()

        # Инициализируем моторы
        self.left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)
        # self.middle_right_motor = Motor(Port.A)
        # self.middle_left_motor = Motor(Port.D)

        # Инициализируем датчики цвета
        self.line_sensor = ColorSensor(Port.S1)
        self.line_sensor_1 = ColorSensor(Port.S2)
        self.line_sensor_2 = ColorSensor(Port.S3)
        

        # Инициализируем привод моторов
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=62.4, axle_track=195)
        self.robot.settings(straight_speed=self.default_speed)

        # Инициализируем датчик касания
        self.touch_sensor_1 = None

        try:
            self.touch_sensor_1 = TouchSensor(Port.S4)
            
        except:
            pass


    # Инициализация переменных
    def init_vars(self):
        
        self.color_list = [0]*10 # Список из 10 элементов(для записи цветов)
        self.run_list = list() # Список для записи кода проезда
        self.d = 62.4 # Диаметр колес

        self.default_speed = 350 # инициализация дефолтной скорости

        self.last_error = 0 # инициализация last_error
        self.error = 0 # инициализация error
        self.I = 0 # инициализация I

        # инициализация цветов
        self.Red = str() 
        self.Yellow = str()
        self.Blue = str()


    # функция для воспроизведения сигнала
    def beep(self, note): 
        self.ev3.speaker.beep(note)



    # функция для остановки двигателей
    def stop(self): 
        self.robot.stop()


    # чтение значения с левого датчика
    def read_line_1(self):
        return self.line_sensor_1.reflection()

    
    # чтение значения с правого датчика
    def read_line_2(self):
        return self.line_sensor_2.reflection()


    # функция для ожидания нажатия датчика касания
    def wait_pressed(self):
        self.ev3.speaker.beep(500,100)
        while not self.touch_sensor_1.pressed(): print(self.line_sensor.rgb())
        wait(300)


    # функция для движения вперед(на вход принимает: расстояние и скорость)
    def forward(self, dist, speed=None):
        if speed:
            self.robot.settings(straight_speed=speed)

        self.stop()
        self.robot.straight(dist)
        self.stop()

        self.robot.settings(straight_speed=self.default_speed)


    # функция для поворота направо(на вход принимает: скорость)
    def right(self, speed=300):
        if speed:
            self.robot.settings(turn_rate=speed)

        self.stop()
        self.robot.turn(90)
        self.stop()
       
        self.robot.settings(turn_rate=300)


    # функция для поворота налево(на вход принимает: скорость)
    def left(self, speed=300):
        if speed:
            self.robot.settings(turn_rate=speed)

        self.stop()
        self.robot.turn(-90)
        self.stop()
       
        self.robot.settings(turn_rate=300)




    # линия без цикла(на вход принимает: скорость, коэффициеты: p, d, i)
    # используется как основа для функций проезда по линии
    def run_line(self, speed=350, kp=2, kd=20, ki=0.02):
        
        self.error = self.line_sensor_1.reflection() - self.line_sensor_2.reflection()
        P = kp * self.error
        D = kd * (self.error - self.last_error)
        self.I += ki * self.error

        U = P + D + self.I
        
        # self.robot.drive(speed, U)

        self.left_motor.run(speed + U)
        self.right_motor.run(speed - U)
        
        self.last_error = self.error

        wait(10)









    # линия по энкодеру(на вход принимает: расстояние, скорость, коэффициеты: p, d, i)
    def run_line_dist(self, dist, speed=350, kp=2.5, kd=50, ki=0.07):
        self.robot.reset()
        self.stop()

        while self.robot.distance() <= dist:
            self.run_line(speed=speed, kp=kp, ki=ki, kd=kd)

        self.stop()



    # линия по перекрестку(на вход принимает: кол-во перекрестков, скорость, коэффициеты: p, d, i)
    def drive_line_per(self, n, speed, kp, kd, ki, forward_drive=1):

        for _ in range(n):
            self.I = 0

            while self.line_sensor_1.reflection() > 30 or self.line_sensor_2.reflection() > 30:
                self.run_line(speed=speed, kp=kp, ki=ki, kd=kd)
                if ki > 10:
                    ki = 0
                

            self.stop()

            if forward_drive:
                self.forward(30)
                self.stop()


    

    # выравнивание(на вход принимает: скорость, пороговый угол)
    def align(self, speed=None, threshold=20):

        if not speed:
            speed = self.default_speed

        self.stop()

        first_sns = 0

        def backward_sensor():
            while 1:
                if self.read_line_1() < threshold and first_sns != -1:
                    self.stop()
                    return -1
                if self.read_line_2() < threshold and first_sns != 1:
                    self.stop()
                    return 1

                self.left_motor.run(-speed)
                self.right_motor.run(-speed)

        first_sns = backward_sensor()
        start_deg = self.left_motor.angle()

        backward_sensor()
        self.stop()

        end_deg = self.left_motor.angle()


        delta_deg = (end_deg - start_deg) * 1.9

        while abs(self.left_motor.angle()-end_deg) < abs(delta_deg):
            self.left_motor.run(-speed*first_sns)
            self.right_motor.run(speed*first_sns)

        self.stop()

        self.forward(20)

        while self.read_line_1() > threshold or self.read_line_2() > threshold:
            l_1 = self.read_line_1() > threshold
            l_2 = self.read_line_2() > threshold

            if l_1 and l_2:
                self.left_motor.run(-speed)
                self.right_motor.run(-speed)
            elif l_1 and not l_2:
                self.left_motor.run(-speed)
                self.right_motor.run(0)
            elif not l_1 and l_2:
                self.left_motor.run(0)
                self.right_motor.run(-speed)

        self.stop()


    # выравнивание "вперёд"(на вход принимает: скорость, пороговый угол)
    def line_align_forward(self, v, thereshold=30): 
        while self.line_sensor_1.reflection() > thereshold or self.line_sensor_2.reflection() > thereshold:
            self.left_motor.run(v)
            self.right_motor.run(v)
            if self.line_sensor_1.reflection() < thereshold:
                while self.line_sensor_1.reflection() < thereshold:
                    self.left_motor.run(-v)
                    self.right_motor.run(-v/2)
                wait(100)
            if self.line_sensor_2.reflection() < thereshold:
                while self.line_sensor_2.reflection() < thereshold:
                    self.left_motor.run(-v/2)
                    self.right_motor.run(-v)
                wait(100)
            wait(10)
        self.stop()



    # подсчет кол-ва кубиков(на вход принимает: расстояние от первого до последнего кубика, скорость)
    # реботает в режиме rgb
    def get_count_сolor_objects(self, dist=560, speed=180):
        now_n = 0
        cur_obj = False

        self.robot.reset()

        while self.robot.distance() <= dist:
            self.run_line(speed=speed)
                
                # примеры кортежей 
                # значения без объекта: (4, 3, 7) (5, 5, 9)
                # blue (1 3 16), (0 4 17)
                # red (9 1 2), (10 0 1), (11 1 2)
                # yellow (14 6 9), (15 7 10)

            # добавляет значение цвета кубика в место в списке, где робот считал кубик, 
            # если значение красного > 8 или значение синего > 15 и кубик не был считан
            if (self.line_sensor.rgb()[0] >= 11 or self.line_sensor.rgb()[2] >= 20) and cur_obj == False:
                cur_obj = True
                now_n += 1
                # self.color_list[(self.robot.distance() // 45) - 1] = self.line_sensor.rgb()
                # print(self.color_list)
                # белый (14, 15, 26)
                # красный (46, 10, 4)
                # синий (8, 20, 42)
                # желтый (63, 48, 8)

            else:
                if cur_obj == True:
                    cur_obj = False
                    print(self.line_sensor.rgb())
                    print((self.robot.distance() // 45) - 1)

                else:
                    ...


        print('*********************') 
        
        self.robot.stop()

        return now_n, self.color_list # возвращает кол-во объектов, список цветов(кортежи или 0)
        
    # Считывание цвета кубика из списка и сравнение с цветом из текстового файла  
    def col_check(self):
        # открытие файла и запись содержимого в список
        x = open('colors.txt','r') 
        convert_rgb = lambda s: list(map(int, s.strip().split(' '))) 
        data_list = x.readlines()
        
        # присваивание переменным значения из полученного списка
        self.Red = convert_rgb(data_list[0])
        self.Yellow = convert_rgb(data_list[1])
        self.Blue = convert_rgb(data_list[2])
        

        for i in range(10):
            if self.color_list[i]:
                red, green, blue = self.color_list[i]
                if blue > int(self.Blue[2])-2:
                    self.run_list.append(1)
                elif blue < int(self.Red[2])+2:
                    self.run_list.append(2)
                else:
                    self.run_list.append(-1)               
            else:
                self.run_list.append(0)
        tiny_font = Font(size=8)
        self.ev3.screen.set_font(tiny_font)
        self.ev3.screen.print(self.run_list)
        return self.run_list




            
    def run_away(self):
        self.robot.stop()
        for num in range(10):
            self.robot.turn(self.run_list[num] * 90)
            self.robot.stop()
            self.drive_line_per(1, 400, 8, 50, 0)




    def check_value(self):

            print(self.line_sensor.rgb())
            wait(5000)

    # main (для использования выше написанных функций)
    def main(self) -> None:
        self.wait_pressed()     
        while self.line_sensor_1.reflection() > 30 and self.line_sensor_2.reflection() > 30:
            self.left_motor.run(200)
            self.right_motor.run(200)
        self.robot.stop()
        self.robot.straight(50)
        self.robot.stop()
        
        
        print(self.get_count_сolor_objects())
        print(self.col_check())
        self.wait_pressed() 
        self.drive_line_per(1, 300, 1, 10, 0.01)   
        self.run_away()



r = Robot()
r.main()
