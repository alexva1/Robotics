import py_trees
import numpy as np
import RobotDART as rd
from src.events import ReachTarget, ToggleFingers, Lift


from src.services.utils import create_grid, create_problems

dt = 0.001 # you are NOT allowed to change this
simulation_time = 60.0 # you are allowed to change this
total_steps = int(simulation_time / dt)

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create robot
robot = rd.Franka(int(1. / dt))
init_position = [0., np.pi / 4., 0., -np.pi / 4., 0., np.pi / 2., 0., 0.04, 0.04]
robot.set_positions(init_position)

max_force = 5.
robot.set_force_lower_limits([-max_force, -max_force], ["panda_finger_joint1", "panda_finger_joint2"])
robot.set_force_upper_limits([max_force, max_force], ["panda_finger_joint1", "panda_finger_joint2"])
#########################################################
robot.set_actuator_types("servo") # you can use torque here

#########################################################
# DO NOT CHANGE ANYTHING IN HERE
# Create boxes
box_positions = create_grid()

box_size = [0.04, 0.04, 0.04]

# Red Box
# Random cube position
red_box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0]
red_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.9, 0.1, 0.1, 1.0], "red_box")

# Green Box
# Random cube position
green_box_pt = np.random.choice(len(box_positions))
while green_box_pt == red_box_pt:
    green_box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0]
green_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.9, 0.1, 1.0], "green_box")

# Blue Box
# Random cube position
box_pt = np.random.choice(len(box_positions))
while box_pt == green_box_pt or box_pt == red_box_pt:
    box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[box_pt][0], box_positions[box_pt][1], box_size[2] / 2.0]
blue_box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [0.1, 0.1, 0.9, 1.0], "blue_box")
#########################################################

boxes = {
    'red': red_box,
    'blue': blue_box,
    'green': green_box
}

#########################################################
# PROBLEM DEFINITION
# Choose problem
problems = create_problems()
problem_id = np.random.choice(len(problems))
problem = problems[problem_id]

print('We want to put the', problem[2], 'cube on top of the', problem[1], 'and the', problem[1], 'cube on top of the', problem[0], 'cube.')
#########################################################

#########################################################
# Create Graphics
gconfig = rd.gui.Graphics.default_configuration()
gconfig.width = 1280 # you can change the graphics resolution
gconfig.height = 960 # you can change the graphics resolution
graphics = rd.gui.Graphics(gconfig)

# Create simulator object
simu = rd.RobotDARTSimu(dt)
simu.set_collision_detector("fcl") # you can use bullet here
simu.set_control_freq(100)
simu.set_graphics(graphics)
graphics.look_at((0., 4.5, 2.5), (0., 0., 0.25))
simu.add_checkerboard_floor()
simu.add_robot(robot)
simu.add_robot(red_box)
simu.add_robot(blue_box)
simu.add_robot(green_box)
#########################################################


# Create tree root
root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
# Create sequence node (for sequential targets)
sequence = py_trees.composites.Sequence(name="Sequence", memory=True)

# open hand
open_hand1 = ToggleFingers(robot, 'open', 'Open Hand 1')
open_hand2 = ToggleFingers(robot, 'open', 'Open Hand 2')
open_hand3 = ToggleFingers(robot, 'open', 'Open Hand 3')

# close hand
close_hand1 = ToggleFingers(robot, 'close', 'Close Hand 1')
close_hand2 = ToggleFingers(robot, 'close', 'Close Hand 2')

# lift object
lift1 = Lift(robot, dt, action='close', offset=0.4, name='Lift 1')
lift2 = Lift(robot, dt, action='open', offset=0.4, name='Lift 2')
lift3 = Lift(robot, dt, action='close', offset=0.4, name='Lift 3')
lift4 = Lift(robot, dt, action='open', offset=0.4, name='Lift 3')


reach_target_1_1 = ReachTarget(robot, dt, boxes[problem[0]], box_name=problem[0]+'_box', action='close', offset=0.04, name="Reach Second Target 1")
reach_target_1_2 = ReachTarget(robot, dt, boxes[problem[0]], box_name=problem[0]+'_box', action='close', offset=0.08, name="Reach Second Target 2")
reach_target_2 = ReachTarget(robot, dt, boxes[problem[1]], box_name=problem[1]+'_box', action='open', offset=0., name="Reach First Target")
reach_target_3 = ReachTarget(robot, dt, boxes[problem[2]], box_name=problem[2]+'_box', offset=0.,  action='open', name="Reach Third Target")


# add events to sequence node
sequence.add_child(reach_target_2)
sequence.add_child(close_hand1)
sequence.add_child(lift1)
sequence.add_child(reach_target_1_1)
sequence.add_child(open_hand1)
sequence.add_child(lift2)
sequence.add_child(reach_target_3)
sequence.add_child(close_hand2)
sequence.add_child(lift3)
sequence.add_child(reach_target_1_2)
sequence.add_child(open_hand3)
sequence.add_child(lift4)

root.add_child(sequence)

for step in range(total_steps):
    if (simu.schedule(simu.control_freq())):
        root.tick_once()
        
    if (simu.step_world()):
        break