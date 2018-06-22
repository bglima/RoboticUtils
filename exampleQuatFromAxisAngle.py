from robutils import *

if __name__ == '__main__':
    # Configuring numpy precision
    np.set_printoptions(precision=4, suppress=True)

    # Defining axis for rotations
    x_axis = np.array([1, 0, 0]).reshape(3, 1)
    y_axis = np.array([0, 1, 0]).reshape(3, 1)
    z_axis = np.array([0, 0, 1]).reshape(3, 1)

    # Traversing axes
    for axis in [x_axis, y_axis, z_axis]:
        print('Rotation around {} axis.'.format(axis.transpose()))
        for i in range(4):
            # Setting axis and angle
            w = axis
            angle = i * math.pi / 2 # 0, 90, 180 and 270 degrees

            # Generating quaternion
            q = quatFromAxisAngle(w, angle)
            print(' - with {} degrees is quat {}'.format(np.rad2deg(angle), q) )
