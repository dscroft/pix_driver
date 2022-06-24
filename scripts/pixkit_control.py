#!/usr/bin/env python3

import sys
import math

import can
import cantools
import os

import rospkg
import rospy
from rospy_message_converter import message_converter
# allow converstion from bool to int and int to bool
#message_converter.ros_to_python_type_map["bool"] += copy.deepcopy( message_converter.python_int_types)

import pix_driver
from pix_driver.msg import *

packagePath = rospkg.RosPack().get_path( "pix_driver" )
dbcFilename = "ros_units.dbc"
dbc_path = os.path.join( packagePath, "src", dbcFilename )

#rospy.get_param( "dbc_path" )
can_type = rospy.get_param( "can_type")
can_channel = rospy.get_param( "can_channel" )

def kill_switch( dbc, candata ):
    maxSteerRaw = 1024  # must be 1024, do not change on pain of pain
    maxSpeedRaw = 600   # must be 600, ditto ^

    # make absolutely certain that NO steering values over maxSteerRaw
    # or speed values over 600 get sent, even if someone messes with the dbc file
    if dbc.frame_id == 387:
        if maxSteerRaw < abs( int.from_bytes( candata[4:6], "little", signed=True ) ):
            raise ValueError( "Steering out of range" )
        
        if maxSpeedRaw < int.from_bytes( candata[0:2], "little", signed=False ):
            raise ValueError( "Speed out of range" )

    elif dbc.frame_id == 390:
        if maxSteerRaw < abs( int.from_bytes( candata[0:2], "little", signed=True ) ):
            raise ValueError( "Front steering out of range" )

        if maxSteerRaw < abs( int.from_bytes( candata[6:8], "little", signed=True ) ):
            raise ValueError( "Rear steering out of range" )

    elif dbc.frame_id == 392:
        if maxSpeedRaw < int.from_bytes( candata[0:2], "little", signed=False ):
            raise ValueError( "Front left speed out of range" )

        if maxSpeedRaw < int.from_bytes( candata[2:4], "little", signed=False ):
            raise ValueError( "Rear left speed out of range" )
            
        if maxSpeedRaw < int.from_bytes( candata[4:6], "little", signed=False ):
            raise ValueError( "Front right speed out of range" )
            
        if maxSpeedRaw < int.from_bytes( candata[6:8], "little", signed=False ):
            raise ValueError( "Rear right speed out of range" )

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
    kill_switch( dbc, candata )

    # send can frame
    bus.send( can.Message( arbitration_id=dbc.frame_id, data=candata ) )

def main():
    rospy.init_node( "pixkit_control", log_level=rospy.DEBUG )

    db = cantools.database.load_file( dbc_path )

    # === create a publisher for each message type ===
    subs = {}
    filters = []
    for i in db.messages:
        # only subscribe to message that go to the VCU
        if "VCU" in i.senders: continue

        try:
            dbc = db.get_message_by_frame_id( i.frame_id )

            limits = {}
            for signal in dbc.signals:
                if signal.name.lower() in ( "steering", "speed", "braking" ):
                    limits[signal.name] = ( signal.minimum, signal.maximum )

            subs[i.frame_id] = rospy.Subscriber( f"~{i.name}", \
                                getattr( pix_driver.msg, f"{i.name}_stamped" ), \
                                lambda m: send_can_callback( bus, dbc, m.data, limits ) )

            filters.append( {"can_id": i.frame_id, 
                             "can_mask": 0x1FFFFFFF, 
                             "extended": i.is_extended_frame} )
        except AttributeError:  # no corresponding msg for this frame
            rospy.logwarn( f"No msg for {i.name}" )

    # === open can device ===
    try:
        bus = can.interface.Bus( channel=can_channel, bustype=can_type,
                    can_filters=filters )
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
