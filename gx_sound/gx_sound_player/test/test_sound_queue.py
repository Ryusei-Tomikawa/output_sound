# -*- coding: utf-8 -*-


from nose.tools import assert_equal

import rospy
from actionlib.server_goal_handle import ServerGoalHandle
from actionlib.status_tracker import StatusTracker
from actionlib_msgs.msg import GoalID
from gx_sound_msgs.msg import SoundRequestActionGoal, SoundRequestGoal

from gx_sound_player.sound_player import SoundQueue


def create_goal(goal_id, secs, file_path):
    gh = ServerGoalHandle(
        StatusTracker(
            goal=SoundRequestActionGoal(
                goal_id=GoalID(id=goal_id, stamp=rospy.Time(secs=secs)),
                goal=SoundRequestGoal(
                    stamp=rospy.Time(secs),
                    file=file_path
                )
            ),
        )
    )
    return gh


g1 = create_goal("1", 3, "a.wav")
g2 = create_goal("2", 2, "b.wav")
g3 = create_goal("3", 5, "c.wav")


class TestSoundQueue(object):
    def test_none(self):
        queue = SoundQueue()
        goals = queue.get_before_time(rospy.Time(0))
        assert_equal(goals, [])

    def test_get_before_time(self):
        goals = [g1, g2, g3]
        queue = SoundQueue()
        for goal in goals:
            queue.append(goal)
        assert_equal(len(queue), 3)

        result = queue.get_before_time(rospy.Time(4))
        assert_equal(len(result), 2)
        assert_equal(result[0].goal.goal.file, "b.wav")
        assert_equal(result[1].goal.goal.file, "a.wav")
        assert_equal(len(queue), 1)

        result = queue.get_before_time(rospy.Time(6))
        assert_equal(len(result), 1)
        assert_equal(result[0].goal.goal.file, "c.wav")
        assert_equal(len(queue), 0)

    def test_max_len(self):
        queue = SoundQueue(max_len=2)
        goals = [g1, g2, g3]
        for goal in goals:
            queue.append(goal)
        assert_equal(len(queue), 2)

        # 直近の再生分ではなく、未来の分がはみ出るべき
        result = queue.get_before_time(rospy.Time(6))
        assert_equal(len(result), 2)
        assert_equal(result[0].goal.goal.file, "b.wav")
        assert_equal(result[1].goal.goal.file, "a.wav")
        assert_equal(len(queue), 0)
