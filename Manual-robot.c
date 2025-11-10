from cyberpi import *
import mbot2
import gamepad
import time

gripperState = 0

while True :
    mbot2.drive_power((gamepad.get_joystick('Ly') + gamepad.get_joystick('Lx')) * 2, (-1 * gamepad.get_joystick('Ly') + gamepad.get_joystick('Lx')) * 2)
    if gamepad.is_key_pressed('N1'):
        mbot2.motor_set(200, 'M2')
        time.sleep(0.1)
        mbot2.motor_set(-30, 'M2')
        time.sleep(1)
        mbot2.motor_set(0, 'M2')
    if (gamepad.get_joystick('Rx') < 10) :
        val2 = int((gamepad.get_joystick('Ry')+100)*180/200)
        mbot2.servo_set(val2,'S3')
    if gamepad.is_key_pressed('N2'):
        mbot2.servo_set(50,'S3')
    if gamepad.is_key_pressed('N3'):
        mbot2.servo_set(180,'S3')
    if (gamepad.is_key_pressed('N4') and gripperState == 1):
        mbot2.servo_set(135,'S1')
        mbot2.servo_set(45,'S4')
        time.sleep(1)
        gripperState = 0
    if (gamepad.is_key_pressed('N4') and gripperState == 0):
        mbot2.servo_set(45,'S1')
        mbot2.servo_set(135,'S4')
        time.sleep(1)
        gripperState = 1