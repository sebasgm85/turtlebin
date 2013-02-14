import roslib; roslib.load_manifest('move_base_msgs')
import rospy
import actionlib
import move_base_msgs.msg as mbm
import geometry_msgs.msg as gm
import tf.transformations as trans

def _to_quaternion(yaw):
    return gm.Quaternion(*trans.quaternion_from_euler(0, 0, yaw))

def _to_pose(x, y, theta):
    return gm.Pose(gm.Point(x, y, 0), _to_quaternion(theta))

rospy.init_node('test_nav')
client = actionlib.SimpleActionClient('/move_base', mbm.MoveBaseAction)
client.wait_for_server()
goal = mbm.MoveBaseGoal()
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.header.frame_id = '/map'
goal.target_pose.pose = _to_pose( 0.988049293195, 2.1175956352, -0.18007133190746352)
client.send_goal(goal)
client.wait_for_server()
