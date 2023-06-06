import numpy as np

# function for damped pseudo-inverse
def damped_pseudoinverse(jac, l = 0.01):
    m, n = jac.shape
    if n >= m:
        return jac.T @ np.linalg.inv(jac @ jac.T + l*l*np.eye(m))
    return np.linalg.inv(jac.T @ jac + l*l*np.eye(n)) @ jac.T

# function for skew symmetric
def skew_symmetric(v):
    mat = np.zeros((3, 3))
    mat[0, 1] = -v[2]
    mat[1, 0] = v[2]
    mat[0, 2] = v[1]
    mat[2, 0] = -v[1]
    mat[1, 2] = -v[0]
    mat[2, 1] = v[0]

    return mat

# function for Adjoint
def AdT(tf):
    R = tf.rotation()
    T = tf.translation()
    tr = np.zeros((6, 6))
    tr[0:3, 0:3] = R
    tr[3:6, 0:3] = skew_symmetric(T) @ R
    tr[3:6, 3:6] = R

    return tr

# angle wrap to [-pi,pi)
def angle_wrap(theta):
    while theta < -np.pi:
        theta += 2 * np.pi
    while theta > np.pi:
        theta -= 2 * np.pi
    return theta

# angle wrap multi
def angle_wrap_multi(theta):
    if isinstance(theta, list):
        th = theta
        for i in range(len(th)):
            th[i] = angle_wrap(th[i])
        return th
    elif type(theta) is np.ndarray:
        th = theta
        for i in range(theta.shape[0]):
            th[i] = angle_wrap(th[i])
        return th
    return angle_wrap(theta)

# enforce joint limits
def enforce_joint_limits(robot, joint_positions, threshold = 0.01):
    positions = np.copy(joint_positions)
    upper_limits = robot.position_upper_limits()
    lower_limits = robot.position_lower_limits()
    for i in range(joint_positions.shape[0]):
        if positions[i] > (upper_limits[i] - threshold):
            positions[i] = upper_limits[i] - threshold
        elif positions[i] < (lower_limits[i] + threshold):
            positions[i] = lower_limits[i] + threshold
    return positions

def create_grid(box_step_x=0.05, box_step_y=0.05):
    box_positions = []
    box_x_min = 0.3
    box_x_max = 0.7
    box_y_min = -0.4
    box_y_max = 0.4

    box_nx_steps = int(np.floor((box_x_max-box_x_min) / box_step_x))
    box_ny_steps = int(np.floor((box_y_max-box_y_min) / box_step_y))

    for x in range(box_nx_steps+1):
        for y in range(box_ny_steps+1):
            box_x = box_x_min + x * box_step_x
            box_y = box_y_min + y * box_step_y
            # if (np.linalg.norm([box_x, box_y]) < 1.):
            #     continue
            box_positions.append((box_x, box_y))

    return box_positions


def create_problems():
    cubes = ['red', 'green', 'blue']

    problems = []

    for cubeA in cubes:
        for cubeB in cubes:
            if cubeB == cubeA:
                continue
            for cubeC in cubes:
                if cubeC == cubeA or cubeC == cubeB:
                    continue
                problems.append([cubeA, cubeB, cubeC])
    
    return problems