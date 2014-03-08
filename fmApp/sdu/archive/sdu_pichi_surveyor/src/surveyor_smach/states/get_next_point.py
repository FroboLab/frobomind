import rospy
import smach
import smach_ros

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal
        TODO: Must be implemented as a service interfacing to a file
    """
    def __init__(self,point_list):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], output_keys=['next_x','next_y'])
        self.ptr = 0
        self.point_list = point_list

    def execute(self, userdata):
        if len(self.point_list) > 0 :
            rospy.loginfo("Setting goal to point %d: (%f,%f) ", self.ptr, self.point_list[self.ptr].x, self.point_list[self.ptr].y)
            userdata.next_x = self.point_list[self.ptr].x
            userdata.next_y = self.point_list[self.ptr].y
            self.ptr = self.ptr + 1
            if self.ptr > len(self.point_list) - 1 :
                self.ptr = 0
            return 'succeeded'  
        else :
            return 'aborted'  