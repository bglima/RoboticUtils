from robutils import *

if __name__ == '__main__':
    # Configuring numpy precision
    np.set_printoptions(precision=4, suppress=True)

    # Defining quaternions to test
    quats = []
    quats.append([0.7071, 0.7071, 0, 0])   # Quaternion that represents 90 degrees around X
    quats.append([0.7071, 0, 0.7071, 0])   # Quaternion that represents 90 degrees around Y
    quats.append([0.7071, 0, 0, 0.7071])   # Quaternion that represents 90 degrees around Z

    quats.append([0.7071, -0.7071, 0, 0])   # Quaternion that represents 90 degrees around -X
    quats.append([0.7071, 0, -0.7071, 0])   # Quaternion that represents 90 degrees around -Y
    quats.append([0.7071, 0, 0, -0.7071])   # Quaternion that represents 90 degrees around -Z

    # Printing output rotation matrices
    for q in quats:
        print( '{} \n'.format(rotMatrixFromQuat(q)) )
