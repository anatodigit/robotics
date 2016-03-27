import RPi.GPIO as gpio
import time

def init():
    gpio.getmode(gpio.BOARD)
    gpio.setup(7,gpio.OUT)
    gpio.setup(11,gpio.OUT)
    gpio.setup(13,gpio.OUT)
    gpio.setup(15,gpio.OUT)
    
def motor_control(tf,pin7,pin11,pin13,pin15)
    init()
    gpio.output(7, pin7)
    gpio.output(11, pin11)
    gpio.output(13, pin13)
    gpio.output(15, pin15)
    time.sleep(tf)
    gpio.cleanup()
    
def forward(tf):
    motor_control(tf,False,True,True,False)
    
def reverse(tf)
    motor_control(tf,True,False,False,True)

def turn_left(tf)
    motor_control(tf,True,True,True,False)

def turn_right(tf)
    motor_control(tf,False,True,False,False)
    
def pivot_left(tf)
    motor_control(tf,True,False,True,False)

def pivot_right(tf)
    motor_control(tf,False,True,False,True)




