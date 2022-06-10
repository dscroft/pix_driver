#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from pix_driver.msg import pix_control
import std_msgs.msg
from dbc import decode_dbc
import can


dbc_path = rospy.get_param("dbc_path")
can_type = rospy.get_param("can_type")
can_channel = rospy.get_param("can_channel")

ms = decode_dbc(dbc_path)
m = ms.get_message_by_name('Auto_control')
code = m.encode({'Steering': 0})

bus = can.interface.Bus(bustype=can_type, channel=can_channel, bitrate=500000)

pub = rospy.Publisher('canbus_message', Int16MultiArray, queue_size=1)

def callback(data):
    speed = max( 0.0, data.Speed ) / 0.0277778 # ms to kmh then /10
    steer = -data.Steer * 1955.69664 # (57.2958 * 1024) / 30
    brake = max( 0.0, min( 1.0, data.Brake ) ) * 1024

    try:
        can_message = m.encode({
            'Speed': int(round(speed)),
            'Steering': int(round(steer)),
            'Braking': int(round(brake)),
            'Gear_shift': data.Gear,
            'EPB': int(data.Handbrake),
            'right_light': int(data.RightLight),
            'left_light': int(data.LeftLight),
            'Front_light': int(data.Light),
            'self_driving_enable': int(data.SelfDrive),
            'Speed_mode': data.SpeedMode,
            'Advanced_mode': 0,
            'mode_selection': data.SteerMode,
            'State_control': int(data.Emergency)
            })
        can_message = [387] + can_message

    except:
        can_message = [387] + 8*[0]

    send_message = can.Message(arbitration_id=387, data=can_message[1:], is_extended_id=False)
    bus.send(send_message)
    message = Int16MultiArray(data=can_message)
    rospy.loginfo(message.data)
    pub.publish(message)



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("control_cmd", pix_control, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
