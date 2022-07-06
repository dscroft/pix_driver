#!/usr/bin/env python3

import sys
import math

import can
import cantools
import os

import rospkg
import rospy
from rospy_message_converter import message_converter
from std_msgs.msg import Int16MultiArray
# allow converstion from bool to int and int to bool
#message_converter.ros_to_python_type_map["bool"] += copy.deepcopy( message_converter.python_int_types)

import pix_driver
from pix_driver.msg import *

packagePath = rospkg.RosPack().get_path( "pix_driver" )
dbcFilename = "ros_units.dbc"
dbc_path = os.path.join( packagePath, "docs", dbcFilename )

#rospy.get_param( "dbc_path" )
can_type = rospy.get_param( "can_type")
can_channel = rospy.get_param( "can_channel" )
legacySupport = rospy.get_param( "legacy", True )

def kill_switch( frameId, candata ):
    """ make absolutely certain that NO steering values over 1024
        or speed values over 600 get sent, even if someone messes with the dbc file
        I have good reasons to believe that this can damage the vehicle, but 
        no definite proof """

    maxSteerRaw = 1024  # must be 1024, do not change on pain of pain
    maxSpeedRaw = 600   # must be 600, ditto ^

    if frameId == 387:
        if maxSteerRaw < abs( int.from_bytes( candata[4:6], "little", signed=True ) ):
            raise ValueError( "Steering out of range" )
        
        if maxSpeedRaw < int.from_bytes( candata[0:2], "little", signed=False ):
            raise ValueError( "Speed out of range" )

    elif frameId == 390:
        if maxSteerRaw < abs( int.from_bytes( candata[0:2], "little", signed=True ) ):
            raise ValueError( "Front steering out of range" )

        if maxSteerRaw < abs( int.from_bytes( candata[6:8], "little", signed=True ) ):
            raise ValueError( "Rear steering out of range" )

    elif frameId == 392:
        if maxSpeedRaw < int.from_bytes( candata[0:2], "little", signed=False ):
            raise ValueError( "Front left speed out of range" )

        if maxSpeedRaw < int.from_bytes( candata[2:4], "little", signed=False ):
            raise ValueError( "Rear left speed out of range" )
            
        if maxSpeedRaw < int.from_bytes( candata[4:6], "little", signed=False ):
            raise ValueError( "Front right speed out of range" )
            
        if maxSpeedRaw < int.from_bytes( candata[6:8], "little", signed=False ):
            raise ValueError( "Rear right speed out of range" )

def legacy_callback( bus, dbc, rosdata ):

    rospy.logdebug( f"legacy callback {rosdata}" )
    try:
        candata = dbc.encode( { "Speed": rosdata[0],
                        "Steering": rosdata[1],
                        "Braking": rosdata[2],
                        "Gear": rosdata[3],
                        "Handbrake": rosdata[4],             
                        "RightLight": rosdata[5],
                        "LeftLight": rosdata[6],
                        "FrontLight": rosdata[7],
                        "SelfDrive": rosdata[8],
                        "SpeedMode": rosdata[9],
                        "Advanced": rosdata[10],
                        "SteerMode": rosdata[11],
                        "Emergency": rosdata[12] } )
        
        rospy.logdebug( candata )

        kill_switch( 387, candata )

        bus.send( can.Message( arbitration_id=387, \
                        data=candata ) )

    except ( ValueError, cantools.database.errors.EncodeError ) as err:
        rospy.logwarn_throttle_identical( 5, str(err) )

def send_can_callback( bus, dbc, rosdata, limits={} ):
    #rospy.loginfo( rosdata )

    # convert ros message to dictionary
    data = message_converter.convert_ros_message_to_dictionary( rosdata )

    # I am using strict mode to encode the can frames so it is 
    # not possible to send values greater than those listed in the dbc 
    # but, I don't want to have to do steering and velocity limiting 
    # code every single time I write a node that controls the pixkit.
    # So to compromise, if an out of bounds steering or velocity value is
    # recieved, constrain it to the limits.
    for field, ( mn, mx ) in limits.items():
       data[field] = min( mx, max( mn, data[field] ) )

    # convert dictionary to can frame
    candata = dbc.encode( data, strict=True )

    # throw exception if the message that is about to be sent could 
    # cause damage
    kill_switch( dbc.frame_id, candata )

    # send can frame
    bus.send( can.Message( arbitration_id=dbc.frame_id, data=candata ) )

def main():
    rospy.init_node( "pixkit_control", log_level=rospy.DEBUG )

    db = cantools.database.load_file( dbc_path )

    # === create a publisher for each message type ===
    subs = {}
    for i in db.messages:
        # only subscribe to message that go to the VCU
        if "VCU" in i.senders: continue

        try:
            dbc = db.get_message_by_frame_id( i.frame_id )

            limits = {}
            for signal in dbc.signals:
                if signal.name.lower() in ( "steering", "speed", "braking" ):
                    limits[signal.name] = ( signal.minimum, signal.maximum )

            subs[i.name] = rospy.Subscriber( f"~{i.name}", \
                                getattr( pix_driver.msg, f"{i.name}_stamped" ), \
                                lambda m: send_can_callback( bus, dbc, m.data, limits ) )

        except AttributeError:  # no corresponding msg for this frame
            rospy.logwarn( f"No control msg for {i.name}" )

    # === backwards compatibility ===
    # this is here to provide support for the original 16bit int array format that
    # was described in the code actually provided by pix
    if legacySupport:
        dbc = db.get_message_by_name( "pix_control" )

        legacySub = rospy.Subscriber( "control_cmd", \
                        Int16MultiArray, 
                        lambda m: legacy_callback( bus, dbc, m.data ) )

    # === open can device ===
    try:
        bus = can.interface.Bus( channel=can_channel, bustype=can_type,
                    can_filters=[] )
    except OSError:
        rospy.logerr( f"Failed to open {can_channel}" )
        return 1

    rospy.spin()

    bus.shutdown()

    return 0

if __name__ == '__main__':
    #try:
    #    main()
    #except rospy.ROSInternalException:
    #    pass
    sys.exit( main() )
