import sys
import rospy
from poppy_torso_control.srv import *
from poppy_control_utils import *

GROUP = 'r_arm'

offset = [0.0, 0.0, 0.0, 0.1]

rospy.wait_for_service('/poppy_offset_movement')
rospy.wait_for_service('/poppy_forward_kinematics')
rospy.wait_for_service('/poppy_collision_distance')
offset_movement = rospy.ServiceProxy('/poppy_offset_movement', OffsetMovement)
forward_kinematics = rospy.ServiceProxy('/poppy_forward_kinematics', ForwardKinematics)
collision_distance = rospy.ServiceProxy('/poppy_collision_distance', CollisionDistance)

resp_collision_distance = collision_distance(GROUP, True, offset)
print('resp_collision_distance:')
print(resp_collision_distance)
print('')

resp_offset_movement = offset_movement(GROUP, offset, True, True) # Execute, Wait
print('resp_offset_movement:')
print(resp_offset_movement)
print('')

resp_forward_kinematics = forward_kinematics(GROUP, resp_offset_movement.target_pos)
print('resp_forward_kinematics:')
print(resp_forward_kinematics)
print('')
