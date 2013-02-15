'''
Extracted from kobuki_auto_docking
'''
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import roslib; roslib.load_manifest('kobuki_msgs')
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from task_manager.task_base import TaskBase

class Dock(TaskBase):
    """
    Docks the Turtlebot2 in the docking base
    """

    goal_class = None
    
    def __init__(self, handle):
        '''
        input_keys=[]
        outcomes = ['succeeded', 'failed', 'preempted', 'error']
        '''
        TaskBase.__init__(self, handle)

    def run(self):
        rospy.sleep(2)
        self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
        rospy.loginfo("waiting for auto_docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo("Sending auto_docking goal and waiting for result")
        self._client.send_goal(goal)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if self.preempt_requested():
            #    self.move_base_client.cancel_goal()
            #        self.service_preempt()
            #        return 'preempted'
            state = self._client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("auto_docking succeeded")
                return 'succeeded'
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("state was:" + str(state))
                return 'failed'
            r.sleep()
