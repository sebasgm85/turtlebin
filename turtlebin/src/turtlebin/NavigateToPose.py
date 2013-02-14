'''
Action for moving the base. Mostly copied from sushi/pr2_python/base.py
'''
import rospy
import actionlib
import tf.transformations as trans
import geometry_msgs.msg as gm
from actionlib_msgs.msg import GoalStatus
import subprocess
import math

# baaaaaad hack!!! move_base_msgs isn't catknized, and so to get this working
# for the binfrastucture hackathon, we need this hack. apologies to the programming gods..
import roslib; roslib.load_manifest('move_base_msgs')
import move_base_msgs.msg as mbm

from task_manager.task_base import TaskBase

class NavigateToPose(TaskBase):
    """
    Navigates to a 2d pose.  Possible outcomes:
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    * 'preempted' Goal was preempted.
    * 'error' Serious error occurred.
    """

    goal_class = None
    
    def __init__(self, handle):
        '''
        input_keys=['frame_id', 'x', 'y', 'theta', 'collision_aware']
        outcomes = ['succeeded', 'failed', 'preempted', 'error']
        '''

        TaskBase.__init__(self, handle)
        move_base_uri = '/move_base'
        self.move_base_node_name = rospy.get_param('move_base_node_name', '/move_base_node')
        self.move_base_client = actionlib.SimpleActionClient(move_base_uri, mbm.MoveBaseAction)
        rospy.loginfo("waiting for move base server")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move base server found")
        # What is this for? If we need it, it needs to be updated to Turtlebot topic: cmd_vel_mux/input/teleop
        # self.cmd_vel_pub = rospy.Publisher("base_controller/command", gm.Twist)

    def run(self):
        goal = self.get_goal()
        frame_id = goal['frame_id']
        x = goal['x']
        y = goal['y']
        theta = goal['theta']

        goal = mbm.MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose = _to_pose(x, y, theta)

        rospy.loginfo("Sending base goal (%f, %f, %f) and waiting for result" % (x, y, theta))
        self.move_base_client.send_goal(goal)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if self.preempt_requested():
            #    self.move_base_client.cancel_goal()
            #        self.service_preempt()
            #        return 'preempted'
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("navigation succeeded")
                return 'succeeded'
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("state was:" + str(state))
                return 'failed'
            r.sleep()

def _yaw(q):
    e = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]

def _to_quaternion(yaw):
    return gm.Quaternion(*trans.quaternion_from_euler(0, 0, yaw))

def _to_pose(x, y, theta):
    return gm.Pose(gm.Point(x, y, 0), _to_quaternion(theta))
