import rospy
import smach
import smach_ros
import tf
import math

from tf import TransformListener, TransformBroadcaster

class getStepTowardsPoint(smach.State):
    """
    Calculates the next step towards a point defined in userdata.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['next_x','next_y'],output_keys=['step_next_x','step_next_y'])
        self.__listen = TransformListener()
        self.__cur_pos = None
        self.__step_next_pos = None
        self.__odom_frame = rospy.get_param("~odom_frame","/odom")
        self.__base_frame = rospy.get_param("~base_frame","/base_footprint")
        self.step = 1
    
    def __get_current_position(self):
        ret = False
        try:
            (self.__cur_pos,rot) = self.__listen.lookupTransform( self.__odom_frame,self.__base_frame,rospy.Time(0))
            ret = True
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
        return ret

    def __get_next_step_pos(self, next):
        # Find direction
        self.__get_current_position()
        print next, self.__cur_pos
        diff = [0, 0]
        diff[0] = next[0] - self.__cur_pos[0] # See mee!!
        diff[1] = next[1] - self.__cur_pos[1]
        # Find unit vector and scale according to step
        magnitude = math.sqrt(diff[0]*diff[0]+diff[1]*diff[1])
        diff = [self.step*diff[0]/magnitude, self.step*diff[1]/magnitude ]
        print diff

        # Add to current point
        self.__step_next_pos = [0, 0]
        self.__step_next_pos[0] = self.__cur_pos[0] + diff[0] # See mee!
        self.__step_next_pos[1] = self.__cur_pos[1] + diff[1]
        
    def execute(self, userdata):
        next = [userdata.next_x, userdata.next_y]
        self.__get_next_step_pos(next)
        userdata.step_next_x = self.__step_next_pos[0]
        userdata.step_next_y = self.__step_next_pos[1]
        return 'succeeded'
        
