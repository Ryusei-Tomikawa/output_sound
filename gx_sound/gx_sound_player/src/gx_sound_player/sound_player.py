# -*- coding: utf-8 -*-

import os
import shlex
import subprocess
import threading

import actionlib
import rospy
from gx_sound_msgs.msg import SoundRequestAction, SoundRequestResult


class SoundQueue(object):
    u""" 再生リクエストのキュー

    :param list[actionlib.server_goal_handle.ServerGoalHandle] _queue: キュー
    """
    def __init__(self, max_len=100):
        self._queue = []
        self._lock = threading.Lock()
        self._max_len = max_len

    def __len__(self):
        return len(self._queue)

    def append(self, goal):
        u""" キューにゴールを追加する

        :type goal: actionlib.server_goal_handle.ServerGoalHandle
        """
        with self._lock:
            self._queue.append(goal)
            self._reorder()

    def remove(self, goal_id):
        u""" 特定の Goal ID を持つリクエストをキューから削除する

        :param str goal_id: 削除するリクエストのゴールID ("/sound_request_client_node-2-1485951939.628" のような書式)
        """
        with self._lock:
            for idx in xrange(len(self._queue)):
                if self._queue[idx].goal.goal_id.id == goal_id:
                    self._queue.pop(idx)
                    return True
        return False

    def _reorder(self):
        u""" タイムスタンプ順に並べ替える """
        self._queue = sorted(self._queue, key=lambda gh: gh.goal.goal.stamp)[:self._max_len]

    def get_before_time(self, stamp):
        u""" 再生開始時刻が stamp 以前のリクエストをタイムスタンプ順に全て取得する

        :rtype list[actionlib.server_goal_handle.ServerGoalHandle]
        """
        with self._lock:
            sep_idx = len(self._queue)
            for idx, gh in enumerate(self._queue):
                if gh.goal.goal.stamp > stamp:
                    sep_idx = idx
                    break
            requests = self._queue[:sep_idx]
            self._queue = self._queue[sep_idx:]
            return requests


class SoundPlayerNode(object):
    u""" wav, ogg ファイルを再生するノード

    :param subprocess.Popen _current_process: 現在実行中のプロセス
    :param actionlib.server_goal_handle.ServerGoalHandle _current_gh: 現在 active な GoalHandle
    """

    RATE = 30

    def __init__(self):
        rospy.init_node('~sound_player')
        self.device_name = rospy.get_param("~device_name")
        self._sound_queue = SoundQueue()
        self._current_gh = None
        self._current_process = None
        self.server = actionlib.ActionServer('~sound_request', SoundRequestAction,
                                             self._callback_goal, self._callback_cancel, False)

    @staticmethod
    def _valid_sound(sound_path):
        u""" 指定されたパスにオーディオファイルが存在すれば True

        :param sound_path: オーディオファイルのフルパス
        :rtype: bool
        :return: wav か ogg 形式のオーディオファイルならば True
        """
        if not os.path.isfile(sound_path):
            rospy.logwarn('File not found: %s' % sound_path)
            return False
        if not sound_path.endswith(('wav', 'ogg')):
            rospy.logwarn('Invalid file type: %s' % sound_path)
            return False
        return True

    def _play_sound(self, gh):
        u""" 指定されたパスのオーディオファイルを再生する

        :param actionlib.server_goal_handle.ServerGoalHandle gh: 再生リクエスト
        """
        sound_path = gh.goal.goal.file
        command = ""
        if sound_path.endswith('wav'):
            # plughw は rate の調整などを自動で行ってくれる
            command = "aplay -D {} {}".format(self.device_name, sound_path)
        elif sound_path.endswith('ogg'):
            command = "ogg123 -d alsa -o dev:{} {}".format(self.device_name, sound_path)
        if command:
            try:
                self._current_gh = gh
                self._current_process = subprocess.Popen(shlex.split(command))
            except Exception as e:
                rospy.logerr('Failed to play sound. Executed command is "{}". {}'.format(command, e))
            else:
                if self._current_process.returncode:
                    rospy.logwarn('Failed to play sound. Executed command is "{}"'.format(command))

    def _play(self):
        u""" キューに再生したいオーディオが存在すれば再生する """
        now = rospy.get_rostime()
        goal_handles = self._sound_queue.get_before_time(now)
        if not goal_handles:
            if self._current_process:
                retcode = self._current_process.poll()
                if retcode is not None:
                    # 再生完了した場合は SUCCEEDED
                    result = SoundRequestResult(result=SoundRequestResult.SUCCEED)
                    self._current_gh.set_succeeded(result)
                    self._current_process = None
                    self._current_gh = None
            return

        # 同時刻に複数ファイルが再生されようとした場合は最後の1つのみが採用される
        for gh in goal_handles[:-1]:
            # 実行前に却下された場合 は REJECTED
            result = SoundRequestResult(result=SoundRequestResult.INTERRUPTED)
            text = "file is not played because of another goal at the same time"
            gh.set_rejected(result, text)

        # 最後の1つを再生する
        gh = goal_handles[-1]
        sound_path = gh.goal.goal.file

        # 再生できなかった場合は ABORTED
        if not (sound_path and self._valid_sound(sound_path)):
            result = SoundRequestResult(result=SoundRequestResult.INTERRUPTED)
            text = "file {} not found".format(sound_path)
            gh.set_aborted(result, text)
            return

        # 現在再生中のファイルがある場合は中断する
        if self._current_process:
            self._current_process.kill()

            # 実行中に中断された場合は PREEMPTED
            result = SoundRequestResult(result=SoundRequestResult.INTERRUPTED)
            text = "playing file is interrupted by new goal"
            self._current_gh.set_canceled(result, text)

            self._current_gh = None
            self._current_process = None

        try:
            gh.set_accepted()
            self._play_sound(gh)
        except Exception as e:
            rospy.logwarn(e)
            result = SoundRequestResult(result=SoundRequestResult.INTERRUPTED)
            gh.set_aborted(result, str(e))

    def _callback_goal(self, gh):
        u""" ゴール受信時のコールバック

        :type gh: actionlib.server_goal_handle.ServerGoalHandle
        """
        self._sound_queue.append(gh)
        rospy.logdebug("New request is appended: {}".format(gh.goal))

    def _callback_cancel(self, gh):
        u""" キャンセル時のコールバック

        :type gh: actionlib.server_goal_handle.ServerGoalHandle
        """
        self._sound_queue.remove(gh.goal.goal.goal_id.id)

    def main(self):
        self.server.start()
        r = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self._play()
            r.sleep()
