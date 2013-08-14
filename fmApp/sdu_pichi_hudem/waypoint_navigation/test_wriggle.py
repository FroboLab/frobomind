from wriggle import Wriggle
import math


def run_update(wriggle, pose_yaw):
    (state, angular_speed) = wriggle.update()
    has_penalty = wriggle.has_sensor_penalty()
    print 'State %d: Speed: %.5f Pose_yaw: %.5f  Penalty: %i' % (state, angular_speed, pose_yaw, has_penalty)
    return angular_speed

def fix_angle(angle):
    if (angle > math.pi):
        angle -= 2*math.pi
    elif (angle < math.pi * -1):
        angle += 2*math.pi
    return angle

turn_angle_left = 1.0
turn_angle_right = -1.0
turn_max_speed = 0.2
speed_gain = 100
pose_yaw = 3.0 # test with values near 0 and pi
pose_east = 0.0
sensor_penalty_distance = 1

test = Wriggle(turn_angle_left, turn_angle_right, turn_max_speed, speed_gain, sensor_penalty_distance)

test.pose_update(0,0,pose_yaw)

print 'Idle'
pose_yaw += run_update(test, pose_yaw)
pose_yaw += run_update(test, pose_yaw)

print 'Set goal'
test.start_wriggle()
for x in xrange(1, 31):
    pose_yaw += run_update(test, pose_yaw)
    pose_yaw = fix_angle(pose_yaw)
    test.pose_update(pose_east,0,pose_yaw)


# Test penalty by moving the robot
print 'Moving east with step 0.2'
for x in xrange(1, 11):
    pose_east += 0.2
    pose_yaw += run_update(test, pose_yaw)
    test.pose_update(pose_east,0,pose_yaw)
