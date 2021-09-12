#! /usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import os
import rospy
from std_msgs.msg import Bool
from gx_sound_msgs.msg import SoundRequestAction, SoundRequestGoal
from gx_sound_msgs.srv import *


def main():
    rospy.init_node('sound_request_client_node')

    # client
    client = actionlib.SimpleActionClient('/gx_sound_player/sound_player/sound_request', SoundRequestAction)
    client.wait_for_server()
    rospy.loginfo("connected to actionlib server")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    now = rospy.get_rostime()

    # 逆順に送信しても時刻順に再生される
    goal1 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 1.0), file=os.path.join(base_dir, "woman_person_detect.wav"))
    goal2 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 6.0), file=os.path.join(base_dir, "man_person_detect.wav"))
    # goal3 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 1.0), file=os.path.join(base_dir, "audio1.wav"))

    # Fill in the goal here
    rospy.loginfo("send request")
    client.send_goal(goal1)
    client.send_goal(goal2)
    # client.send_goal(goal3)
    rospy.loginfo("waiting for result")
    # 5s wait 
    client.wait_for_result()
    # client.wait_for_result(rospy.Duration.from_sec(10.0))

    result = client.get_result()
    rospy.loginfo(result)


if __name__ == '__main__':
    main()
