#!/usr/bin/env python3

""" here to provide support for the same multiarray format message that 
   is used by the offical pix_drive but with added bounds limiting to 
   ensure no hardware issues """

import can
import cantools
import os

import software_fuse

import rospkg
import rospy
import std_msgs.msg

packagePath = rospkg.RosPack().get_path( "pix_driver" )
dbcFilename = "legacy.dbc"
dbcPath = os.path.join( packagePath, "docs", dbcFilename )

canType = rospy.get_param( "can_type" )
canChannel = rospy.get_param( "can_channel" )

def legacy_callback( bus, dbc, pub, rosdata ):
    rospy.logdebug( f"ros data: {rosdata}" )
    try:
        data = { "Speed": rosdata[0],
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
                 "Emergency": rosdata[12] }
        rospy.logdebug( f"data: {data}" )

        candata = dbc.encode( data )
        rospy.logdebug( f"candata: {candata}" )

        software_fuse.check( 387, candata )

        bus.send( can.Message( arbitration_id=dbc.frame_id, 
                               data=candata, 
                               is_extended_id=dbc.is_extended_frame ) )

        message = std_msgs.msg.Int16MultiArray( data=candata )
        pub.publish( message )

    except ( ValueError, cantools.database.errors.EncodeError ) as err:
        rospy.logwarn_throttle_identical( 5, str(err) )

def main():
    rospy.init_node( "listener", anonymous=True, log_level=rospy.INFO )

    db = cantools.database.load_file( dbcPath )

    dbc = db.get_message_by_frame_id( 387 )

    # === open can device ===
    try:
        bus = can.interface.Bus( channel=canChannel, bustype=canType,
                    can_filters=[] )
    except OSError:
        rospy.logerr( f"Failed to open {canChannel}" )
        return 1

    pub = rospy.Publisher( "canbus_message", std_msgs.msg.Int16MultiArray, queue_size=1 )

    sub = rospy.Subscriber( "control_cmd", std_msgs.msg.Int16MultiArray, lambda m: legacy_callback( bus, dbc, pub, m.data ) )
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
