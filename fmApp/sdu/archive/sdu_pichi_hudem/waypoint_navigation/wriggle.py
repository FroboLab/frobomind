import math
#from enum import Enum

class Turn():
    def __init__(self, turn_max_speed, speed_gain, allowed_angle_error):
        self.turn_max_speed = turn_max_speed
        self.speed_gain = speed_gain
        self.allowed_angle_error = allowed_angle_error
        self.pose_yaw = False
        self.goal_yaw = False
        self.angular_speed_step = self.turn_max_speed / 20
        self.last_angular_speed = 0

    def diff_angles(self, angle1, angle2):
        diff = angle1 - angle2
        if (diff > math.pi):
            diff -= 2 * math.pi
        elif (diff < math.pi * -1):
            diff += 2 * math.pi
        return diff

    def pose_update (self, easting, northing, yaw):
	self.pose_yaw = yaw

    def doTurn(self, angle):
        print 'New goal %f' % angle
        self.goal_yaw = angle

    def stop(self):
        self.goal_yaw = False

    def update(self):
        angle_error = 0
        turn_done = True
        angular_speed = 0.0
        desired_angular_speed = 0.0
        if (self.goal_yaw != False):
            angle_error = self.diff_angles(self.goal_yaw,self.pose_yaw)
            if (math.fabs(angle_error) < self.allowed_angle_error):
                self.stop()
            else:
                turn_done = False
                desired_angular_speed = angle_error * self.speed_gain
                if (desired_angular_speed > self.turn_max_speed):
                    desired_angular_speed = self.turn_max_speed
                elif (desired_angular_speed < (self.turn_max_speed * -1)):
                    desired_angular_speed = self.turn_max_speed * -1
        else:
            desired_angular_speed = 0.0

        # acc limit
        angular_speed = desired_angular_speed
        speed_step = self.angular_speed_step
        if angular_speed < 0:
            speed_step = speed_step * -1
        
        if math.fabs(angular_speed) > math.fabs(self.last_angular_speed):
            angular_speed = angular_speed + speed_step
            if math.fabs(angular_speed) > math.fabs(desired_angular_speed):
                angular_speed = desired_angular_speed

        self.last_angular_speed = angular_speed

        return (turn_done, angular_speed)

        
class WriggleState():
    DONE = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TURN_CENTER = 4

class Wriggle():
    def __init__(self, turn_angle_left, turn_angle_right, turn_max_speed, speed_gain, sensor_penalty_distance):
        self.pose = False
        self.wrigglepose = False # Pose where the robot wriggles
        self.turn_angle_left = turn_angle_left
        self.turn_angle_right = turn_angle_right
        self.sensor_penalty_distance = sensor_penalty_distance
        self.sensor_penalty_done = True
        self.state = WriggleState.DONE
        self.is_turning = False
        allowed_angle_error = 0.05
        self.turn = Turn(turn_max_speed, speed_gain, allowed_angle_error)

        self.goal_angle_left = 0.0
        self.goal_angle_right = 0.0
        self.goal_angle_center = 0.0
    def fix_angle(self,angle):
        if (angle > math.pi):
            angle -= 2*math.pi
        elif (angle < math.pi * -1):
            angle += 2*math.pi
        return angle

    def set_goal_angles(self):
        self.goal_angle_left = self.fix_angle(self.pose[2] + self.turn_angle_left)
        self.goal_angle_right = self.fix_angle(self.pose[2] + self.turn_angle_right)
        self.goal_angle_center = self.pose[2]

    def turn_left(self):
        self.state = WriggleState.TURN_LEFT
        self.turn.doTurn(self.goal_angle_left)
        print 'Turn left %f' % self.goal_angle_left

    def turn_right(self):
        self.state = WriggleState.TURN_RIGHT
        self.turn.doTurn(self.goal_angle_right)
        print 'Turn right'

    def turn_center(self):
        self.state = WriggleState.TURN_CENTER
        self.turn.doTurn(self.goal_angle_center)
        print 'Turn center'

    def start_wriggle(self):
        if not self.has_sensor_penalty():
            # Start turning
            self.set_goal_angles()
            self.turn_left()

            # Needed for sensor distance penalty
            self.wrigglepose = self.pose
            self.sensor_penalty_done = False
    def is_done(self):
        if (self.state == WriggleState.DONE):
            return True
        else:
            return False

    def pose_update (self, easting, northing, yaw):
        self.turn.pose_update(easting, northing, yaw)
	self.pose = [easting, northing, yaw]

    def has_sensor_penalty(self):
        if (self.sensor_penalty_done):
            return False
        else:
            # Test for distance
            dist_easting = self.pose[0] - self.wrigglepose[0]
            dist_northing = self.pose[1] - self.wrigglepose[1]
            distance_from_wrigglepose = math.sqrt(dist_easting*dist_easting + dist_northing*dist_northing)
            if (distance_from_wrigglepose > self.sensor_penalty_distance):
                self.sensor_penalty_done = True
                return False
            else:
                return True

    def update(self):
        angular_speed = 0.0
        if (self.state != WriggleState.DONE):
            (turn_done, angular_speed) = self.turn.update()
            # If current turn is done, change state
            if (turn_done == True):
                print 'Turning done'
                if (self.state == WriggleState.TURN_LEFT):
                    print 'Turning right'
                    self.turn_right()
                elif (self.state == WriggleState.TURN_RIGHT):
                    print 'Turning center'
                    self.turn_center()
                elif (self.state == WriggleState.TURN_CENTER):
                    print 'Wriggle done'
                    self.state = WriggleState.DONE
                    
        return (self.state, angular_speed)
                
        
