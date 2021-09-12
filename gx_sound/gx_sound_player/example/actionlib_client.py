#! /usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import os
import rospy
from std_msgs.msg import Bool
from gx_sound_msgs.msg import SoundRequestAction, SoundRequestGoal
from gx_sound_msgs.srv import *

cnt = 0

# callback
def Callback(sound_req):

    sound_res = sound_signalResponse()

    print("Succesful! Called Sound Server")
    print("sound_req.stop_signal :=%d", sound_req)

    # client
    client = actionlib.SimpleActionClient('/gx_sound_player/sound_player/sound_request', SoundRequestAction)
    client.wait_for_server()
    rospy.loginfo("connected to actionlib server")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    now = rospy.get_rostime()

    global cnt

    # Stop Signal
    if sound_req.stop_signal and cnt == 0:

        # 逆順に送信しても時刻順に再生される
        goal1 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 1.0), file=os.path.join(base_dir, "woman_stop.wav"))
        # Fill in the goal here
        rospy.loginfo("send request")
        client.send_goal(goal1)
        rospy.loginfo("waiting for result")

        # 5s wait 
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)
        cnt += 1
        
        if result:
            sound_res.end_signal = True
    
    elif sound_req.avoid_signal and cnt == 1:
        # 逆順に送信しても時刻順に再生される
        goal1 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 1.0), file=os.path.join(base_dir, "woman_avoid.wav"))
        # Fill in the goal here
        rospy.loginfo("send request")
        client.send_goal(goal1)
        rospy.loginfo("waiting for result")

        # 5s wait 
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)
        cnt += 1
        
        if result:
            sound_res.end_signal = True
    
    elif sound_req.comeback_signal and cnt == 2:
        # 逆順に送信しても時刻順に再生される
        goal1 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 1.0), file=os.path.join(base_dir, "woman_comeback.wav"))
        # Fill in the goal here
        rospy.loginfo("send request")
        client.send_goal(goal1)
        rospy.loginfo("waiting for result")

        # 5s wait 
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)
        cnt = 0
        
        if result:
            sound_res.end_signal = True


    return sound_res


def main():
    rospy.init_node('sound_request_client_node')

    print('wait for manager signal ...')

    #sound service server
    sound_service = rospy.Service('sound_signal', sound_signal, Callback)

    rospy.spin()


if __name__ == '__main__':
    main()
