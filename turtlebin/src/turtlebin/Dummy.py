import rospy
from task_manager.task_base import TaskBase

class Dummy(TaskBase):
    goal_class = None

    def __init__(self, handle):
        TaskBase.__init__(self, handle)

    def run(self):
        rospy.loginfo('Dummy activity running')
        
