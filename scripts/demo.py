#!/usr/bin/env python3
import rospy
import math

from pix_driver.msg import *

maxSpeed = 2
maxSteer = math.radians(30)

if __name__ == "__main__":
    rospy.init_node( "demo", anonymous=True )

    publisher = rospy.Publisher( "/pixkit_control/Auto_control", Auto_control_stamped, queue_size=1 )

    rate = rospy.Rate( 20 )
	
    command = Auto_control()
    command.Speed = 0
    command.Steering = 0
    command.Braking = 0
    command.Gear = Auto_control.Gear_Neutral
    command.SpeedMode = Auto_control.SpeedMode_Slow
    command.SteerMode = Auto_control.SteerMode_Opposite

    command.Handbrake = Auto_control.Handbrake_Off
    command.RightLight = Auto_control.Light_Off
    command.LeftLight = Auto_control.Light_Off
    command.FrontLight = Auto_control.Light_On
    command.SelfDrive = Auto_control.SelfDrive_On
    command.Emergency = Auto_control.Emergency_Off

    speedDir = -0.01
    steerDir = -0.01    

    while not rospy.is_shutdown():
        if command.Speed >= maxSpeed or command.Speed <= 0:
            speedDir *= -1
        command.Speed += speedDir

        if abs( command.Steering ) > maxSteer:
            steerDir *= -1
        command.Steering += steerDir

        stamped = Auto_control_stamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.data = command
        
        publisher.publish( stamped )
            
        rate.sleep()
