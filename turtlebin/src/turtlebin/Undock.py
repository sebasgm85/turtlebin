'''
Made up from scratch
'''
import rospy
from geometry_msgs.msg import Twist, Vector3
import roslib; roslib.load_manifest('kobuki_msgs')
from kobuki_msgs.msg import MotorPower
from task_manager.task_base import TaskBase

class Undock(TaskBase):
    """
    Undocks the Turtlebot2 from the docking station by teleoping it backwards a foot or two
    """

    goal_class = None
    bw_twist = Twist(linear=Vector3( -0.15, 0, 0))
    
    def __init__(self, handle):
        '''
        input_keys=[]
        outcomes = ['succeeded', 'failed', 'preempted', 'error']
        '''
        TaskBase.__init__(self, handle)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)
        self.motors_pub = rospy.Publisher("/mobile_base/commands/motor_power", MotorPower)
        rospy.loginfo("created cmd_vel publisher")

    def run(self):
        rospy.loginfo("Resetting motors, just in case")
        self.motors_pub.publish(MotorPower(state=1))
        rospy.sleep(1)

        rospy.loginfo("Sending 8 backwards-motion twists at 4hz and hoping it will work")
        r = rospy.Rate(4)
        for i in range(8):
            self.cmd_vel_pub.publish(self.bw_twist)
            r.sleep()

        return 'succeeded'
