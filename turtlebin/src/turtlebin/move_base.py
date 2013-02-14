import rospy
from geometry_msgs.msg import Twist
from topic_tools.srv import MuxSelect

def make_twist(forward, left):
    '''
    Args:
        forward (float): How fast to move forward in m/s. Negative for reverse.
        left (float): How fast to left in rad/s. Negative for right turn.
    '''
    tw = Twist()
    tw.linear.x = forward
    tw.angular.z = left
    return tw

class MoveBase:
    def __init__(self, cmd_vel_topic='/nav/cmd_vel'):
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)
        
        # Tell the cmd_vel_mux to use our topic. Assumes that our
        # topic is already in the list that cmd_vel_mux has.
        mux_topic = '/cmd_vel_mux/select'
        rospy.wait_for_service(mux_topic)
        mux_serv = rospy.ServiceProxy(mux_topic, MuxSelect)
        mux_serv(cmd_vel_topic)
        rospy.loginfo('Set cmd_vel_mux to %s' % cmd_vel_topic)

    def set_vel(self, twist):
        self.cmd_vel_pub.publish(twist)
