import unittest
import numpy as np
from math import sin, cos, pi
from robot_kinematics import rotation_matrix, translation_matrix, prev_to_curr_joint_transform_matrix, forward_kinematics, Point, InverseKinematics

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

    # example forward kinematics matrixes
    f_kine_0 = np.matlib.array([[0.5,0.5,0.70710678,3.41421356],
                                [0.5,0.5,-0.70710678,3.41421356],
                                [-0.70710678,0.70710678,0,2],
                                [0,0,0,1]])

    f_kine_1 = np.matlib.array([[-0.5,-0.5,0.70710678,0],
                                [-0.5,-0.5,-0.70710678,0],
                                [0.70710678,-0.70710678,0,6.82842712],
                                [0,0,0,1]])
    f_kine_2 = np.matlib.array([[0.5,0.866025,0,4],
                                [0,0,-1,0],
                                [-0.866025,0.5000,0,2],
                                [0,0,0,1]])

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

    def test_forward_kinematics(self):
        tout, _ = forward_kinematics([pi/4, pi/4, -pi/4, -pi/4], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0])
        np.testing.assert_array_almost_equal(self.f_kine_0, tout)
        tout1, _ = forward_kinematics([pi/4, pi/4, pi/4, pi/4], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0])
        np.testing.assert_array_almost_equal(self.f_kine_1, tout1)
        tout2, _ = forward_kinematics([0, pi/3, -pi/3, -pi/3], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0])
        np.testing.assert_array_almost_equal(self.f_kine_2, tout2)

    def test_point(self):
        p0 = Point([3,3,3])
        p1 = Point([4,6,4])
        pdist = p0.distance_to_point(p1)
        np.testing.assert_almost_equal(pdist, 3.3166247903554) # check first to second distance
        pdist1 = p1.distance_to_point(p0)
        np.testing.assert_almost_equal(pdist1, 3.3166247903554) # check second to first distance
        np.testing.assert_almost_equal(pdist1, pdist) # check both computed distances
        p2 = p0.get_point_between(p1, 3) # check slightly less distance point position
        np.testing.assert_array_almost_equal(np.array([p2.x, p2.y, p2.z]), np.array([3.9045340337332908, 5.713602101199873, 3.9045340337332908]))

    def test_fabrik(self):
        # constants
        dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
        joints_lengths = [2, 2, 2, 2]
        robo_arm_joint_limits = {'x_limits': [0,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
        robo_arm_reach_limit = 6 # lenght of 3 joint is the limit
        first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed
        fkine = InverseKinematics()
        # first position
        dest_point = [2, -2, 4]
        ik_angles = []
        try:
            ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, robo_arm_joint_limits, robo_arm_reach_limit, first_rev_joint_point, 0.001, 100)
            dh_matrix_out = [ik_angles, [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
            fk, _ = forward_kinematics(dh_matrix_out[0], dh_matrix_out[1], dh_matrix_out[2], dh_matrix_out[3])
        except Exception as e:
            print(e)
        forward_dest_point = [fk[0,3], fk[1,3], fk[2,3]]
        max_decimal_error = 3 # set max decimail error to the same accuracy as IK, 3 decmial places
        np.testing.assert_array_almost_equal(np.array(dest_point), np.array(forward_dest_point), max_decimal_error)
        # second position
        dest_point = [1, -4, 5]
        ik_angles = []
        try:
            ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, robo_arm_joint_limits, robo_arm_reach_limit, first_rev_joint_point, 0.001, 100)
            dh_matrix_out = [ik_angles, [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
            fk, _ = forward_kinematics(dh_matrix_out[0], dh_matrix_out[1], dh_matrix_out[2], dh_matrix_out[3])
        except Exception as e:
            print(e)
        forward_dest_point = [fk[0,3], fk[1,3], fk[2,3]]
        max_decimal_error = 3 # set max decimail error to the same accuracy as IK, 3 decmial places
        np.testing.assert_array_almost_equal(np.array(dest_point), np.array(forward_dest_point), max_decimal_error)

def suite():
    suite = unittest.TestSuite()
    suite.addTest(forward_kinematics_unittests('test_rotation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_translation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_transformation_matrix'))
    suite.addTest(forward_kinematics_unittests('test_prev_to_curr_joint_transform_matrix'))
    suite.addTest(forward_kinematics_unittests('test_forward_kinematics'))
    suite.addTest(forward_kinematics_unittests('test_point'))
    suite.addTest(forward_kinematics_unittests('test_fabrik'))
    return suite

if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite())