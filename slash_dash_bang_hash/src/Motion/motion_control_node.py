#!/usr/bin/env python
import rospy
import struct
import time
import serial

# ser = serial.Serial('COM11', 115200, timeout=None) #windows
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)
#ser = serial.Serial('/dev/pts/3', 115200, timeout=None) #TEST
from geometry_msgs.msg import Twist, Pose2D, Vector3
# from std_srvs.srv import Trigger, TriggerResponse

from slash_dash_bang_hash.msg import State, MotorSpeeds

import numpy as np
from math import cos, sin, pi

# Kinematic constants
RHO = 0.02875 # Wheel radius [m]
S_SCALE = 0.05
SX1 = S_SCALE*-1.0 # Wheel spin vectors - body frame (unit vectors)
SY1 = S_SCALE*0.0
SX2 = S_SCALE*0.5
SY2 = S_SCALE*0.866
SX3 = S_SCALE*0.5
SY3 = S_SCALE*-0.866
RX1 = 0.0 # Wheel position vectors - body frame (in meters)
RY1 = -0.079
RX2 = -0.06841
RY2 = 0.0395
RX3 = 0.06841
RY3 = 0.0395

theta_ = 0
vel_cmd_ = Vector3()
wheel_speeds_ = np.zeros(shape=(3))
robot_number_ = 1

M_ = (1/RHO)*np.array([[SX1, SY1, (SY1*RX1 - SX1*RY1)],
                      [SX2, SY2, (SY2*RX2 - SX2*RY2)],
                      [SX3, SY3, (SY3*RX3 - SX3*RY3)]])

model_offset_ = 0
dither_pwm_ = 0
dither_period_ = 0.04

# ============== MOTOR FUNCTIONS ====================
def writeFloat(f):
	ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
	return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
	ser.write('p')
	writeFloat(p1)
	writeFloat(p2)
	writeFloat(p3)
def setSpeed(s1, s2, s3):
	ser.write('s')
	writeFloat(s1)
	writeFloat(s2)
	writeFloat(s3)
def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
	ser.write('k')
	ser.write(str(motor))
	writeFloat(p)
	writeFloat(i)
	writeFloat(qpps)
def setAdvancedConstants(motor, offset, dither_pwm, dither_period):
	ser.write('a')
	ser.write(str(motor))
	writeFloat(offset)
	writeFloat(dither_pwm)
	writeFloat(dither_period)
def setT(period_ms, tau_ms):
	ser.write('t')
	writeFloat(period_ms)
	writeFloat(tau_ms)
def getSpeed():
	ser.write('v')
	return readFloat(), readFloat(), readFloat()
def getEncoderCount():
	ser.write('e')
	return readFloat(), readFloat(), readFloat()
def disengage():
	ser.write('d')


# ============== ROS NODE FUNCTIONS ====================
def _handle_robot_state(msg):
    global theta_
    theta_ = msg.theta*pi/180.0;

def _handle_velocity_command(msg):
    global vel_cmd_
    vel_cmd_.x = msg.linear.x
    vel_cmd_.y = msg.linear.y
    vel_cmd_.z = msg.angular.z*pi/180.0
    #print(msg)
    computeMotorSpeeds()

def computeMotorSpeeds():
    global wheel_speeds_

    #Rotate by 90 degrees to align world and robot coordinate frames
    #(Front of robot is x in world frame and y in robot frame)
    ct = cos(theta_-pi/2)
    st = sin(theta_-pi/2)

    R = np.array([[ ct, st, 0],
                  [-st, ct, 0],
                  [  0,  0, 1]])

    velocities = np.array([vel_cmd_.x, vel_cmd_.y, vel_cmd_.z])
    wheel_speeds_ = np.dot(M_, R).dot(velocities)

    #print(wheel_speeds_)
    max_speed = np.amax(np.array([abs(x) for x in wheel_speeds_]))
    #mid_speed = np.median(np.array([abs(x) for x in wheel_speeds_]))
    #print(max_speed)

    #if (mid_speed < 0.2):
    #  wheel_speeds_ = np.array([0, 0, 0])
    #elif mid_speed < 2.0:
    #  wheel_speeds_ = np.array([x*(2.0/mid_speed) for x in wheel_speeds_])
    if (max_speed < 0.2):
      wheel_speeds_ = np.array([0, 0, 0])
    elif max_speed < 2.0:
      wheel_speeds_ = np.array([x*(2.0/max_speed) for x in wheel_speeds_])

    sendVelocityCommands()

def sendVelocityCommands():
    global robot_number_
    global model_offset_
    global dither_pwm_
    global dither_period_
    speedM1 = wheel_speeds_[0] # rot/s
    speedM2 = wheel_speeds_[1] # rot/s
    speedM3 = wheel_speeds_[2] # rot/s

    #print(wheel_speeds_)

    pulsePerRotation = 0
    if robot_number_ == 1:
        # Slash constant
        pulsePerRotation = 4955 #Old motors
    else:
        # Bang constant
        pulsePerRotation = 116.2 #New motors


    # HACK
    #setAdvancedConstants(1, sign(speedM1)*model_offset_, dither_pwm_, dither_period_)
    #setAdvancedConstants(2, sign(speedM2)*model_offset_, dither_pwm_, dither_period_)
    #setAdvancedConstants(3, sign(speedM3)*model_offset_, dither_pwm_, dither_period_)
    setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)
    #speeds_actual_raw = getSpeed()
    #speeds_actual = [x/pulsePerRotation for x in speeds_actual_raw]
    #print(speeds_actual)

def sign(value):
    if value > 0:
	return 1
    if value < 0:
	return -1
    else:
	return 0


def main():
    global robot_number_
    global model_offset_
    global dither_pwm_
    global dither_period_
    rospy.init_node('MotionControl', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('robot_state', State, _handle_robot_state)
    rospy.Subscriber('vel_command', Twist, _handle_velocity_command)
    motor_speed_pub_ = rospy.Publisher('motor_speeds', MotorSpeeds, queue_size=10)

    robot_number_ = rospy.get_param('~robot_number', 1)



    # Set the PIDQ values for all motors
    if robot_number_ == 1:
        # Slash constants
        setPID(0, 12.5, 0.4, 50000)
        #setPID(1, -1, -0.4, 49000)
        #setPID(2, -1, -0.4, 48000)
        #setPID(3, -1, -0.4, 50000)

        # motor, offset, dither_pwm, dither_period
        setAdvancedConstants(0, 0, 0, 0.01)
    else:
        # Bang constants
        pulsePerRotation = 116.2
        setPID(0, 3.5, 0.0, 6.04*pulsePerRotation)
        #setPID(1, 1.5, 0.5, 49000)
        #setPID(2, 1.5, 0.5, 48000)
        #setPID(3, 1.5, 0.5, 49000)

        # motor, offset, dither_pwm, dither_period
        setAdvancedConstants(0, 0, 0, 0.01)



    # Set tick period (triggers PID control) and velocity filter corner frequency
    setT(20, 100)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
