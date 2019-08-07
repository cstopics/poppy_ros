import sys
import rospy
from poppy_torso_control.srv import *
from poppy_control_utils import *

GROUP = 'r_arm'
FPS = 40

rospy.wait_for_service('/poppy_plan_movement')
predef_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)

rospy.wait_for_service('/poppy_get_end_effector_pos')
get_end_effector = rospy.ServiceProxy('/poppy_get_end_effector_pos', GetEndEffectorPos)

resp = predef_mov(  GROUP,  # group
                    False,      # rand_start
                    True,       # current_start 
                    # [1.2443350553512573, 1.2443350553512573],         # start_pos
                    # [161.29, -18.70],         # start_pos
                    [],
                    True,       # rand_target
                    # [1.2443350553512573, 0.4289175271987915],         # target_pos 
                    # [161.29, -65.42],         # target_pos 
                    [],
                    True,       # execute
                    True,       # wait
                    True,       # ret_plan
                    FPS)         # ret_fps

mov = None
if resp.error==0:
    mov = plan2mov(resp, FPS)
    mov['start_angles'] = list(resp.start_pos)
    mov['target_angles'] = list(resp.target_pos)
    end_pos = get_end_effector(GROUP)
    mov['target_end_effector'] = list(end_pos.xyz)

