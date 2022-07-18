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
dbcPath = os.path.join( packagePath, "docs", dbcFilename )

canType = rospy.get_param( "can_type" )
canChannel = rospy.get_param( "can_channel" )

legacySupport = rospy.get_param( "legacy", True )

def main():
    rospy.init_node( "pixkit_status", log_level=rospy.DEBUG )

    db = cantools.database.load_file( dbcPath )

    # === create a publisher for each message type ===
    pubs = {}
    filters = []
    for i in db.messages:
        # only publish frames from the VCU
        if "VCU" not in i.senders: continue

        # backwards compatibility
        # this is here to provide support for the original status message format that
        # was described in the code actually provided by pix
        if i.name == "pix_feedback" and not legacySupport: continue

        try:
            stamped = i.name not in ("pix_feedback",)
            topicName = f"{i.name}_stamped" if stamped else i.name
            messageType = getattr( pix_driver.msg, topicName )
            p = rospy.Publisher( f"~{i.name}", messageType, queue_size=1 )

            if i.frame_id in pubs:
                pubs[i.frame_id].append( ( stamped, i, p ) )
            else:
                pubs[i.frame_id] = [ ( stamped, i, p ) ]              

            # if we set up filters now we can be confident that every can message we
            # the bus yields later on, does exist, does have ros message and does have
            # a publisher for it. only thing left to worry about is decode errors
            filters.append( {"can_id": i.frame_id, 
                             "can_mask": 0x1FFFFFFF, 
                             "extended": i.is_extended_frame} )

        except AttributeError:  # no corresponding msg for this frame
            rospy.logwarn( f"No status msg for {i.name}" )

    rospy.logdebug( f"filters: {filters}" )

    # === open can device ===
    try:
        bus = can.interface.Bus( channel=canChannel, bustype=canType,
                can_filters=filters )
    except OSError:
        rospy.logerr( f"Failed to open {canChannel}" )
        return 1

    # === main loop ===
    while not rospy.is_shutdown():
        # get can message
        msg = bus.recv( 1 )
        if msg is None:   # nothing recieved
            continue
            
        if msg.arbitration_id not in pubs: continue # why does the filter let some stuff through?

        # set up stamped message
        for stamped, dbc, pub in pubs[msg.arbitration_id]:
            rospy.logdebug( dbc.name )

            # decode raw can bytes to dictionary
            try:
                data = dbc.decode( msg.data )
            except ValueError:
                rospy.logwarn( f"{dbc.name} decoding failed" )

            rospy.logdebug( f"{dbc.name} {data}" )
            
            for k, v in data.items():
            	if isinstance(v, cantools.database.can.signal.NamedSignalValue):
            	    data[k] = v.value

            rospy.logdebug( f"{dbc.name} {data}" )

            if stamped:
                msgType = f"pix_driver/{dbc.name}_stamped"
                data = { "data": data }
            else:
                msgType = f"pix_driver/{dbc.name}"

            message = message_converter.convert_dictionary_to_ros_message( msgType, data, check_types=False, check_missing_fields=False )

            if stamped:
                message.header.stamp = rospy.Time.now()

            pub.publish( message )

            #messageStamped = p.data_class()
            #messageStamped.header.stamp = rospy.Time.now()

            # convert decoded dictionary to ros message
            #messageStamped.data = message_converter.convert_dictionary_to_ros_message( f"pix_driver/{dbc.name}", data, check_types=False, check_missing_fields=True )
            
            #pub.publish( messageStamped )

    bus.shutdown()

if __name__ == '__main__':
    sys.exit( main() )
