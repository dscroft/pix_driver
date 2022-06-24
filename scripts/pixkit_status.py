#!/usr/bin/env python3

import sys

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

def main():
    rospy.init_node( "pixkit_status", log_level=rospy.DEBUG )

    db = cantools.database.load_file( dbc_path )

    # === create a publisher for each message type ===
    pubs = {}
    filters = []
    for i in db.messages:
        # only publish frames from the VCU
        if "VCU" not in i.senders: continue

        try:
            pubs[i.frame_id] = rospy.Publisher( f"~{i.name}", getattr( pix_driver.msg, f"{i.name}_stamped" ), queue_size=1 )

            # if we set up filters now we can be confident that every can message we
            # the bus yields later on, does exist, does have ros message and does have
            # a publisher for it. only thing left to worry about is decode errors
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

    # === main loop ===
    while not rospy.is_shutdown():
        # get can message
        msg = bus.recv( 1 )
        if msg is None:   # nothing recieved
            continue

        # decode raw can bytes to dictionary
        try:
            data = db.decode_message( msg.arbitration_id, msg.data )
            rospy.logdebug( f"{msg.arbitration_id}, {data}" )
        except ValueError:
            rospy.logwarn( f"{db.get_message_by_frame_id( msg.arbitration_id ).name} decoding failed" )

        # set up stamped message
        messageStamped = pubs[msg.arbitration_id].data_class()
        messageStamped.header.stamp = rospy.Time.now()

        # convert decoded dictionary to ros message
        messageStamped.data = message_converter.convert_dictionary_to_ros_message( f"pix_driver/{db.get_message_by_frame_id( msg.arbitration_id ).name}", data, check_types=False, check_missing_fields=True )
        
        pubs[msg.arbitration_id].publish( messageStamped )

    bus.shutdown()

if __name__ == '__main__':
    sys.exit( main() )
