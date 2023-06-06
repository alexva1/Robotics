import py_trees
import numpy as np

class ToggleFingers(py_trees.behaviour.Behaviour):
    counter = 0
    def __init__(self, robot, action, name="ToggleFingers",):
        super(ToggleFingers, self).__init__(name)
        self.robot = robot
        self.action = action 

    def setup(self):
        pass

    def initialise(self):
        pass

    def terminate(self, new_status):
        pass  
          
    def update(self):
        self.counter += 1
        cmd = np.zeros(9)
        if self.action == 'open':
            cmd[7] = 0.2
        else:
            cmd[7] = -0.2
        self.robot.set_commands(cmd)

        if self.counter >= 100:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING