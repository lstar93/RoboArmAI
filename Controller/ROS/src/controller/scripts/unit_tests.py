import unittest
import numpy as np
from sympy import sin, cos, pi
from six_dof_kinematics import rotation_matrix, translation_matrix, prev_to_curr_joint_transform_matrix

class forward_kinematics_unittests(unittest.TestCase):

    theta = pi/2
    rot_x_matrix = np.array([[1,0,0],
                            [0,cos(theta),-sin(theta)],
                            [0,sin(theta),cos(theta)]])
    rot_y_matrix = np.array([[cos(theta),0,sin(theta)],
                            [0,1,0],
                            [-sin(theta),0,cos(theta)]])
    rot_z_matrix = np.array([[cos(theta),-sin(theta),0],
                            [sin(theta),cos(theta),0],
                            [0,0,1]])

    translation_matrix = np.array([[1,0,0,1],
                                   [0,1,0,2],
                                   [0,0,1,3],
                                   [0,0,0,1]])

    transformation_matrix_x = rotation_matrix('x', theta, 4)
    transformation_matrix_x[0,3] = 1
    transformation_matrix_x[1,3] = 2
    transformation_matrix_x[2,3] = 3

    transformation_matrix_y = rotation_matrix('y', theta, 4)
    transformation_matrix_y[0,3] = 1
    transformation_matrix_y[1,3] = 2
    transformation_matrix_y[2,3] = 3

    transformation_matrix_z = rotation_matrix('z', theta, 4)
    transformation_matrix_z[0,3] = 1
    transformation_matrix_z[1,3] = 2
    transformation_matrix_z[2,3] = 3

    el_1 = 3
    el_2 = 5
    dh_test_arr = np.matlib.array([[cos(theta),  sin(theta),      0,        el_2 * cos(theta)], 
                                   [sin(theta), -cos(theta),      0,        el_2 * sin(theta)], 
                                   [     0,            0,        -1,        el_1              ], 
                                   [     0,            0,         0,                1         ]])

    def test_rotation_matrix(self):
        np.testing.assert_array_almost_equal(self.rot_x_matrix, rotation_matrix('x', pi/2))
        np.testing.assert_array_almost_equal(self.rot_y_matrix, rotation_matrix('y', pi/2))
        np.testing.assert_array_almost_equal(self.rot_z_matrix, rotation_matrix('z', pi/2))

    def test_translation_matrix(self):
        np.testing.assert_array_almost_equal(self.translation_matrix, translation_matrix([1, 2, 3]))
        
    # transformation -> translation + rotation
    def test_transformation_matrix(self):
        np.testing.assert_array_almost_equal(self.transformation_matrix_x, translation_matrix([1, 2, 3], 'x', pi/2))
        np.testing.assert_array_almost_equal(self.transformation_matrix_y, translation_matrix([1, 2, 3], 'y', pi/2))
        np.testing.assert_array_almost_equal(self.transformation_matrix_z, translation_matrix([1, 2, 3], 'z', pi/2))

    def test_prev_to_curr_joint_transform_matrix(self):
        np.testing.assert_array_almost_equal(self.dh_test_arr, prev_to_curr_joint_transform_matrix(pi/2, 3, 5, pi))

def suite():
    suite = unittest.TestSuite()
    suite.addTest(forward_kinematics_unittests('test_rotation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_translation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_transformation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_prev_to_curr_joint_transform_matrix'))
    return suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite())