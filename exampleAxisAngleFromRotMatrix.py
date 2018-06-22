from robutils import *

if __name__ == '__main__':
    # Configuring numpy precision
    np.set_printoptions(precision=4, suppress=True)

    # Testing default rotation matrices
    matrices = []
    # Basic rotations
    matrices.append(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])) # Rotation arround X by 90 degree
    matrices.append(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])) # Rotation arround Y by 90 degree
    matrices.append(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])) # Rotation arround Z by 90 degree

    # Rotation with infinite solutions
    matrices.append(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])) # Rotation arround any axis by 0 degree

    # Rotation with double solutions
    matrices.append(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])) # Rotation arround X by 180 degree
    matrices.append(np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])) # Rotation arround Y by 180 degree
    matrices.append(np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])) # Rotation arround Z by 180 degree

    # Convert matrices to axis, angle
    for r in matrices:
        # Convert from matrix to axis and angle
        (axis, angle) = axisAngleFromRotMatrix( r )
        print('axis:{}, angle:{} \n'.format(axis, np.rad2deg(angle)))
