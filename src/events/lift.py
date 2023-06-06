import py_trees
import numpy as np
from src.services.utils import damped_pseudoinverse
from src.controller.PI_Controller import PITask

class Lift(py_trees.behaviour.Behaviour):
    def __init__(self, robot, dt, action, offset=0, name="Lift"):
        super(Lift, self).__init__(name)
        
        self.dt = dt
        self.robot = robot
        self.eef_link_name = 'panda_ee'
        self.offset=offset
        self.action = action 
        
    def setup(self):
        pass

    def initialise(self):
        self.Kp = 2. 
        self.Ki = 0.01 
        self.tf_desired = self.robot.body_pose(self.eef_link_name)
        self.tf_desired.set_translation(self.tf_desired.translation() + [0., 0., self.offset])
        self.controller = PITask(self.tf_desired, self.dt, self.Kp, self.Ki)
        
    def update(self):
        tf = self.robot.body_pose(self.eef_link_name)
        vel = self.controller.update(tf)
        jac = self.robot.jacobian(self.eef_link_name) 
        jac_pinv = damped_pseudoinverse(jac) 
        cmd = jac_pinv @ vel
        if self.action == 'open':
            cmd[7] = 0.2
        else:
            cmd[7] = -0.2
        
        self.robot.set_commands(cmd)

        err = np.linalg.norm(self.controller.error(tf))
        if err < 1e-3:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass