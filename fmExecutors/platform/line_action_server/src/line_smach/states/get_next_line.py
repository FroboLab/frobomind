import rospy, smach, smach_ros

class getNextLine(smach.State):
    """
        Temporary implementation. State giving the next position goal
        TODO: Must be implemented as a service interfacing to a file
    """
    def __init__(self,point_list):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], output_keys=['next_ax','next_ay','next_bx','next_by'])
        self.ptr = 1
        self.point_list = point_list
        
    def execute(self, userdata):
        if len(self.point_list) > 1 :
            rospy.loginfo("Setting goal to line %d: (%f,%f),(%f,%f) ", self.ptr, self.point_list[self.ptr-1].x, self.point_list[self.ptr-1].y,self.point_list[self.ptr].x, self.point_list[self.ptr].y)
            if self.ptr == 0 :
                # Handle special case of last to first line
                ax = userdata.next_ax = self.point_list[len(self.point_list)-1].x
                ay = userdata.next_ay = self.point_list[len(self.point_list)-1].y
            else:
                ax = userdata.next_ax = self.point_list[self.ptr-1].x
                ay = userdata.next_ay = self.point_list[self.ptr-1].y
            bx = userdata.next_bx = self.point_list[self.ptr].x
            by = userdata.next_by = self.point_list[self.ptr].y
            self.ptr = self.ptr + 1
            if self.ptr > len(self.point_list) - 1 :
                self.ptr = 0
            
            return 'succeeded'  
        else :
            rospy.loginfo(rospy.get_name() + " GET_NEXT_LINE aborted because no line was present in list")
            return 'aborted'  