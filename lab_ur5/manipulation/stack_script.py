from lab_ur5.manipulation.utils import distribute_blocks_in_positions
from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2
from lab_ur5.motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.motion_planning.motion_planner import MotionPlanner


block_positions_r2frame = [[-0.7, -0.6, 0.0],
    [-0.7, -0.7, 0.0],
    [-0.7, -0.8, 0.0],
    [-0.7, -0.9, 0.0]]

block_positions_world = [
    [-0.72, -0.395, 0.0],
    [-0.72, -0.495, 0.0],
    [-0.72, -0.595, 0.0],
    [-0.72, -0.695, 0.0]]


target_location_world = [-0.72 ,-0.395]

motion_planner = MotionPlanner()

gt = GeometryAndTransforms.from_motion_planner(motion_planner)

r2_controller = ManipulationController(ur5e_2["ip"], ur5e_2["name"], motion_planner, gt)
r1_controller = ManipulationController(ur5e_1["ip"], ur5e_1["name"], motion_planner, gt)
#r1_controller.plan_and_move_home()

#r2_controller.plan_and_move_home()

#DIDN'T NEED TO USE THIS EVENTUALLY
block_positions_r2frame2 = []
for block_position in block_positions_world:
    bp_r2frame = gt.point_world_to_robot(ur5e_2["name"], block_position)
    block_positions_r2frame.append(bp_r2frame)

distribute_blocks_in_positions(block_positions_world,r2_controller)

r2_controller.stack(block_positions_world,target_location_world)

