'''
Extracted from kobuki_auto_docking
'''
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import roslib; roslib.load_manifest('kobuki_msgs')
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
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
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)

        # The dock_pose is taken from a rosparam which is global and should be set (currently) on everything.launch 
        #TODO use a dictionary with just one param '/dock_pose' and add the '/dock_orientation' to replace Quaternion harcoding 
        if rospy.has_param('/dock_pose/x') and rospy.has_param('/dock_pose/y') and  rospy.has_param('/dock_pose/z'):
            self.dock_pose.x = rospy.get_param('/dock_pose/x')
            self.dock_pose.y = rospy.get_param('/dock_pose/y')
            self.dock_pose.z = rospy.get_param('/dock_pose/z')
        else:
            self.dock_pose.x = -0.0252804756165
            self.dock_pose.y = 2.70252466202
            self.dock_pose.z = 0.0

    def _create_dock_pose(self):
        dock_pose = PoseWithCovarianceStamped()
        dock_pose.header.stamp = rospy.Time.now()
        dock_pose.header.frame_id = "/map"
       # dock_pose.pose.pose.position = Point(-0.0252804756165, 2.70252466202, 0.0)
        dock_pose.pose.pose.position = Point(self.dock_pose.x, self.dock_pose.y, self.dock_pose.z)
        dock_pose.pose.pose.orientation = Quaternion(0.0, 0.0, 0.706556777013, 0.707656357886)
        dock_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        return dock_pose
        

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
                break
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("state was:" + str(state))
                return 'failed'
            r.sleep()

        if  not rospy.is_shutdown():
            rospy.loginfo("Finished docking procedure")
            rospy.sleep(1)

            # Take the opportunity to localize the robot
            rospy.loginfo("Setting initial pose to known dock location")
            self.initial_pose_pub.publish( self._create_dock_pose())
            rospy.sleep(1)
            return 'succeeded'
