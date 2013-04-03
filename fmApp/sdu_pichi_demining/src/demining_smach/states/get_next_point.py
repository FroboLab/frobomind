import rospy
import smach
import smach_ros

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal
        TODO: Must be implemented as a service interfacing to a file
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['next_x','next_y'])
        self.ptr = 0
        #self.position_list =[[-5,-5],[-5,5],[-4,5],[-4,-5],[-3,-5],[-3,5]]
        self.generate_coverage([0,0],[20,20],1)

    def generate_coverage(self, lowerleft, upperright, lanewidth):
        # Start in lower left corner
        numrows = (upperright[1] - lowerleft[1])/lanewidth
        heading = "right"
        self.position_list = []
        for rowpos in range(0,numrows):
            # Define end row points
            rowoffset = lowerleft[1] + lanewidth * rowpos
            point_left = [lowerleft[0],rowoffset]
            point_left_via = [lowerleft[0]-3, rowoffset]
            point_right = [upperright[0],rowoffset]
            point_right_via = [upperright[0]+3, rowoffset]
            
            # Append points to list depending on heading
            if heading == "right":
                self.position_list.append(point_left)
                self.position_list.append(point_right)
                self.position_list.append(point_right_via)
                # Add mid points
                #for colpos in range (lowerleft[0], upperright[0]):
                #    self.position_list.append([colpos,rowpos])
                heading = "left"
            else:
                self.position_list.append(point_right)
                self.position_list.append(point_left)
                self.position_list.append(point_left_via)
                #for colpos in range (upperright[0], lowerleft[0]):
                #    self.position_list.append([colpos,rowpos])
                heading = "right"

    def execute(self, userdata):
        userdata.next_x = self.position_list[self.ptr][0]
        userdata.next_y = self.position_list[self.ptr][1]
        rospy.loginfo("go to point: %d , %d" % (self.position_list[self.ptr][0], self.position_list[self.ptr][1]))
        self.ptr = self.ptr + 1
        if self.ptr == len(self.position_list) - 1 :
            self.ptr = 0
        return 'succeeded'       
