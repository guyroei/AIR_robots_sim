from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2
from lab_ur5.motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.motion_planning.motion_planner import MotionPlanner
from numpy import pi

"this point is located within the boundaries of both robots"
shared_point = [-0.42, -0.6, 0.4]

motion_planner = MotionPlanner()

gt = GeometryAndTransforms.from_motion_planner(motion_planner)

r1_controller = ManipulationController(ur5e_1["ip"], ur5e_1["name"], motion_planner, gt)
r2_controller = ManipulationController(ur5e_2["ip"], ur5e_2["name"], motion_planner, gt)

"reset environment"
r1_controller.release_grasp()
r1_controller.plan_and_move_home()
r2_controller.release_grasp()
r2_controller.plan_and_move_home()


'''
This function moves a block from a pickup position using the first robot, hands it over to the second robot,
and finally places it in the designated location on the table.

Parameters:
pickup_position (list): A list representing the pickup position [x, y, z] of the block.
'''
def block_transfer(pickup_position):
    start_height = 0.15

    "pick up the block with the first robot ur5e_2"
    xyz_src = [pickup_position[0], pickup_position[1], pickup_position[2] + start_height]
    rz1 = 0
    r2_controller.plan_and_move_to_xyzrz(xyz_src[0], xyz_src[1], xyz_src[2], rz1, direction="down")
    r2_controller.pick_up(xyz_src[0], xyz_src[1], rz1, xyz_src[2])

    "move the first robot to the shared point"
    rz2 = 0
    shared_point2 = shared_point #gt.point_world_to_robot(ur5e_2["name"],shared_point)
    r2_controller.plan_and_move_to_xyzrz(shared_point2[0], shared_point2[1], shared_point2[2], rz2, direction="right")
    #r2_controller.release_grasp()

    "move the second robot to the shared point with a small offset to avoid collision"
    rz3 = -pi/2
    offset_shared_point = [shared_point[0] + 0.1, shared_point[1], shared_point[2]]
    r1_controller.plan_and_move_to_xyzrz(offset_shared_point[0], offset_shared_point[1], offset_shared_point[2], rz3, direction="left")
    "transfer the block from the first robot to the second robot"
    r1_controller.release_grasp()
    r1_controller.moveL_relative([-0.08,0,0])
    r1_controller.grasp()
    r2_controller.release_grasp()
    r1_controller.moveL_relative([0.08, 0, 0])
    putdown_position = [0.4, 0 ,0.06] #middle of the target table
    rz4 = -pi/2
    r1_controller.plan_and_move_to_xyzrz(putdown_position[0], putdown_position[1], putdown_position[2], rz4,direction="down")
    r1_controller.put_down(putdown_position[0], putdown_position[1],rz3, putdown_position[2])



if __name__ == "__main__":
    # corner of tale of ur5e_2, where I usually stack blocks for collection by the robot
    pick_up_position_r2frame = (-0.3614, 0.1927,0)

    pickup_position_world = gt.point_robot_to_world(ur5e_2["name"],pick_up_position_r2frame)
    pickup_position_world = [pickup_position_world[0],pickup_position_world[1],0.0]
    block_transfer(pickup_position_world)

