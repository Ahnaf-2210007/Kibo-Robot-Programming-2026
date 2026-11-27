#!/usr/bin/python3
# -*- coding:utf-8 -*-
import json
import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

# ---- Tunable settings ----
START_DELAY = 0.0      # browser e live dekhte chaile 15.0 dao, dekha shesh hole 0.0 koro
TIME_LIMIT = 270.0     # 300s er age safe margin, er por baki checkpoint skip kore goal e jabe
SETTLE_TIME = 1.5      # checkpoint e pouchhe stable hobar jonno wait (second)
DOCK_POSITION = [0.2, 0.0, 0.0]


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action client
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service client
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    def __notify_mission_start(self):
        rospy.wait_for_service('/event/mission_start')
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        rospy.wait_for_service('/event/mission_finished')
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        if rospy.is_shutdown():
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    def __get_checkpoint_id(self, checkpoint):
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __resolve_all_checkpoints(self):
        """Return (list of targets, goalpoint_id)."""
        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s', res.success)
        if not res.success:
            return [], None
        try:
            payload = json.loads(res.message)
            goal_id = payload.get('goalpoint_id')
            targets = []
            for cp in payload.get('checkpoints', []):
                if not isinstance(cp, dict):
                    continue
                cid = self.__get_checkpoint_id(cp)
                pos = cp.get('position')
                quat = (cp.get('attitude') or {}).get('quaternion')
                if cid is None or not pos or not quat:
                    continue
                targets.append({
                    'checkpoint_id': cid,
                    'position': [float(v) for v in pos[:3]],
                    'quaternion': [float(v) for v in quat[:4]],
                })
            return targets, goal_id
        except Exception as e:
            rospy.logwarn('Failed to parse checkpoints payload: %s', e)
            return [], None

    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            if rospy.is_shutdown():
                self.__command.cancel_goal()
                rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__action_active = False

    def __run_user_flow(self, msg):
        try:
            if START_DELAY > 0:
                rospy.sleep(START_DELAY)

            t_start = rospy.get_time()
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: route planning and game flow
            # ***********************************************
            targets, goal_id = self.__resolve_all_checkpoints()
            if not targets:
                rospy.logerr('no checkpoints')
                return

            goal_target = None
            remaining = []
            for t in targets:
                if t['checkpoint_id'] == goal_id:
                    goal_target = t
                else:
                    remaining.append(t)

            current = list(DOCK_POSITION)

            # visit remaining checkpoints, nearest first
            while remaining and not rospy.is_shutdown():
                if rospy.get_time() - t_start > TIME_LIMIT:
                    rospy.logwarn('time is running out, skipping remaining checkpoints')
                    break

                nxt = min(
                    remaining,
                    key=lambda t: sum((a - b) ** 2 for a, b in zip(t['position'], current))
                )
                remaining.remove(nxt)

                if not self.__move_to_checkpoint(nxt):
                    rospy.logwarn('move failed for checkpoint %d', nxt['checkpoint_id'])
                    continue

                current = nxt['position']
                rospy.sleep(SETTLE_TIME)

                if not self.__notify_checkpoint_passed(nxt['checkpoint_id']):
                    rospy.logwarn('retry checkpoint %d', nxt['checkpoint_id'])
                    rospy.sleep(1.0)
                    self.__notify_checkpoint_passed(nxt['checkpoint_id'])

            # finally go to the goal point
            if goal_target is not None and not rospy.is_shutdown():
                if self.__move_to_checkpoint(goal_target):
                    rospy.sleep(SETTLE_TIME)
                    self.__notify_checkpoint_passed(goal_target['checkpoint_id'])
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __stop_processing(self, request):
        try:
            if self.__action_active:
                self.__command.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()