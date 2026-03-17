#!/usr/bin/env python

import configparser,os
import numpy
import math
# import tf
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from ament_index_python.packages import get_package_share_directory
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick


class ComputeCalibState(EventState):
    """
    Output a fixed pose to move.

    <= done									   Pose has been published.
    <= fail									   Task fail and finished

    """
    
    def __init__(self, eye_in_hand_mode, calibration_file_name, customize_file):
        """Constructor"""
        super(ComputeCalibState, self).__init__(outcomes=['finish'], input_keys=['base_h_tool', 'camera_h_charuco'])
        self.eye_in_hand_mode = eye_in_hand_mode
        self.camera_object_list = TransformArray()
        self.world_effector_list = TransformArray()
        #ProxyServiceCaller._initialize(ComputeCalibState._node)
        ProxyServiceCaller.initialize(ComputeCalibState._node)
        self.calib_compute_client = ProxyServiceCaller({'/compute_effector_camera_quick':ComputeEffectorCameraQuick})
        self.save_pwd = get_package_share_directory('charuco_detector') + '/config/hand_eye_calibration/'
        self.config = configparser.ConfigParser()
        self.config.optionxform = str #reference: http://docs.python.org/library/configparser.html

        if customize_file:
            self.calibration_file_name = str(calibration_file_name)
            with open(self.save_pwd+ self.calibration_file_name, 'w') as file:
                self.config.write(file)
        else:
            if eye_in_hand_mode:
                self.calibration_file_name = "eye_in_hand_calibration.ini"
            else:
                self.calibration_file_name = "eye_to_hand_calibration.ini"


    
    def execute(self, userdata):
        req = ComputeEffectorCameraQuick.Request()
        req.camera_object = self.camera_object_list
        req.world_effector = self.world_effector_list
        print ("========================================================================================================")
        res = self.calib_compute_client.call('/compute_effector_camera_quick', req)
        
        print('x = '  + str(res.effector_camera.translation.x))
        print('y = '  + str(res.effector_camera.translation.y))
        print('z = '  + str(res.effector_camera.translation.z))
        print('qx = ' + str(res.effector_camera.rotation.x))
        print('qy = ' + str(res.effector_camera.rotation.y))
        print('qz = ' + str(res.effector_camera.rotation.z))
        print('qw = ' + str(res.effector_camera.rotation.w))

        # config = configparser.ConfigParser()
        # config.optionxform = str #reference: http://docs.python.org/library/configparser.html
        self.config.read(self.save_pwd + self.calibration_file_name)
        # config.read(curr_path + '/config/hand_eye_calibration/'+ self.calibration_file_name)

        if self.config.get("hand_eye_calibration" ,"x") != None:
            pass
        else:
            self.config.add_section("hand_eye_calibration")
        self.config.set("hand_eye_calibration", "x",  str(res.effector_camera.translation.x))
        self.config.set("hand_eye_calibration", "y",  str(res.effector_camera.translation.y))
        self.config.set("hand_eye_calibration", "z",  str(res.effector_camera.translation.z))
        self.config.set("hand_eye_calibration", "qx", str(res.effector_camera.rotation.x))
        self.config.set("hand_eye_calibration", "qy", str(res.effector_camera.rotation.y))
        self.config.set("hand_eye_calibration", "qz", str(res.effector_camera.rotation.z))
        self.config.set("hand_eye_calibration", "qw", str(res.effector_camera.rotation.w))

        with open(self.save_pwd+ self.calibration_file_name, 'w') as file:
            self.config.write(file)
        return 'finish'
    
    def on_enter(self, userdata):
        if self.eye_in_hand_mode:
            print("------------------------------------------------------------------")
            self.world_effector_list = userdata.base_h_tool
            self.camera_object_list = userdata.camera_h_charuco
            # print(self.camera_object_list)
            # print(self.world_effector_list)
        else:
            self.world_effector_list.header = userdata.base_h_tool.header
            print ("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            print (userdata.base_h_tool)

            self.camera_object_list.header = userdata.camera_h_charuco.header

            for transform in userdata.base_h_tool.transforms:
                trans = self.quaternion_matrix([transform.rotation.x, transform.rotation.y,
                                                              transform.rotation.z, transform.rotation.w])
                trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
                trans = self.inverse_matrix(trans)
                trans_B = Transform()
                trans_B.translation.x, trans_B.translation.y, trans_B.translation.z = trans[:3, 3]
                trans_B.rotation.x, trans_B.rotation.y, trans_B.rotation.z, \
                    trans_B.rotation.w = self.quaternion_from_matrix(trans)
                self.world_effector_list.transforms.append(trans_B)

            for transform in userdata.camera_h_charuco.transforms:
                trans = self.quaternion_matrix([transform.rotation.x, transform.rotation.y,
                                                              transform.rotation.z, transform.rotation.w])
                trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
                trans = self.inverse_matrix(trans)
                trans_C = Transform()
                trans_C.translation.x, trans_C.translation.y, trans_C.translation.z = trans[:3, 3]
                trans_C.rotation.x, trans_C.rotation.y, trans_C.rotation.z, \
                    trans_C.rotation.w = self.quaternion_from_matrix(trans)
                self.camera_object_list.transforms.append(trans_C)

    def quaternion_matrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.
    
        >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
        >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
        True
    
        """
        q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
        nq = numpy.dot(q, q)
        if nq < numpy.finfo(float).eps * 4.0:
            return numpy.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = numpy.outer(q, q)
        return numpy.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
            (q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
            (q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
            (0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)
    
    def quaternion_from_matrix(self, matrix):
        """Return quaternion from rotation matrix.
    
        >>> R = rotation_matrix(0.123, (1, 2, 3))
        >>> q = quaternion_from_matrix(R)
        >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
        True
    
        """
        q = numpy.empty((4, ), dtype=numpy.float64)
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[3] = t
            q[2] = M[1, 0] - M[0, 1]
            q[1] = M[0, 2] - M[2, 0]
            q[0] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
        return q
    
    def inverse_matrix(self, matrix):
        """Return inverse of square transformation matrix.
    
        >>> M0 = random_rotation_matrix()
        >>> M1 = inverse_matrix(M0.T)
        >>> numpy.allclose(M1, numpy.linalg.inv(M0.T))
        True
        >>> for size in range(1, 7):
        ...     M0 = numpy.random.rand(size, size)
        ...     M1 = inverse_matrix(M0)
        ...     if not numpy.allclose(M1, numpy.linalg.inv(M0)): print size
    
        """
        return numpy.linalg.inv(matrix)

