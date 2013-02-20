'''
Made up from scratch
'''
import rospy
from task_manager.task_base import TaskBase
from espeak import espeak

class Speak(TaskBase):
    """
    Speaks a text using python esynth
    """

    def __init__(self, handle):
        '''
        input_keys=['text']
        outcomes = ['succeeded', 'failed', 'preempted', 'error']
        '''
        TaskBase.__init__(self, handle)

    def run(self):
        goal = self.get_handle().get_goal()
        text = goal['text']
        rospy.loginfo("Speaking text: %s" % text)
        espeak.synth( text)
        return 'succeeded'
