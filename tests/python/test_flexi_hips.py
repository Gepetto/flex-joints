#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 13:47:16 2022

@author: nvilla
"""

import unittest
import numpy as np
from flex_joints import Flex


class DynamicsTestCase(unittest.TestCase):
    def setUp(self):

        # [LH_pitch, LH_roll, RH_pitch, RH_roll]
        H_stiff = np.array([2200, 2200, 5000, 5000])
        H_damp = 2 * np.sqrt(H_stiff)
        flexToJoint = np.array([0, 0, 0.09])

        settings = dict(
            left_stiffness=H_stiff[:2],
            left_damping=H_damp[:2],
            right_stiffness=H_stiff[:2],
            right_damping=H_damp[:2],
            flexToJoint = flexToJoint,
            dt=0.002,
            MA_duration=0.01,
            left_hip_indices=np.array([0, 1, 2]),
            right_hip_indices=np.array([6, 7, 8]),
            filtered=True,
        )
        flex = Flex()
        flex.initialize(settings)

        encoders_q = np.zeros(12)
        encoders_dq = np.zeros(12)

        self.flex = flex
        self.encoders_q = encoders_q
        self.encoders_dq = encoders_dq
        self.tol = 1e-6

    def test_zero_torque(self):
        desired_torques = np.zeros(12)

        # No torque produce no deflection:
        (corrected_q, corrected_dq) = self.flex.correctEstimatedDeflections(
            desired_torques, self.encoders_q, self.encoders_dq
        )
        self.assertTrue((np.abs(corrected_q - self.encoders_q) < self.tol).all())

        left_flexing_torque = np.zeros(2)
        right_flexing_torque = np.zeros(2)

        (corrected_q, corrected_dq) = self.flex.correctDeflections(
            left_flexing_torque, right_flexing_torque, self.encoders_q, self.encoders_dq
        )
        self.assertTrue((np.abs(corrected_q - self.encoders_q) < self.tol).all())

    def test_persistent_toque(self):

        left_flexing_torque = np.array([10, 60])
        right_flexing_torque = np.array([0, 0])

        (corrected_q, corrected_dq0) = self.flex.correctDeflections(
            left_flexing_torque,
            right_flexing_torque,
            self.encoders_q.copy(),
            self.encoders_dq.copy(),
        )

        self.assertTrue(np.abs(corrected_dq0[2]) >= 0)

        for i in range(10):
            (corrected_q, corrected_dq10) = self.flex.correctDeflections(
                left_flexing_torque,
                right_flexing_torque,
                self.encoders_q.copy(),
                self.encoders_dq.copy(),
            )

        self.assertTrue(np.abs(corrected_dq10[2]) >= 0)
        self.assertTrue(np.abs(corrected_dq10[2]) < np.abs(corrected_dq0[2]))

        for i in range(1000):
            (corrected_q, corrected_dq1000) = self.flex.correctDeflections(
                left_flexing_torque,
                right_flexing_torque,
                self.encoders_q.copy(),
                self.encoders_dq.copy(),
            )

        self.assertTrue(np.abs(corrected_dq1000[2]) >= 0)
        self.assertTrue(np.abs(corrected_dq1000[2]) < self.tol)

        (corrected_q, corrected_dq1001) = self.flex.correctDeflections(
            left_flexing_torque,
            right_flexing_torque,
            self.encoders_q.copy(),
            self.encoders_dq.copy(),
        )
        self.assertTrue(np.abs(corrected_dq1001[2]) < self.tol)

    def test_reset(self):
        self.flex.reset()

        left_flexing_torque = np.random.rand(2) * 100
        right_flexing_torque = np.random.rand(2) * 100

        (corrected_q0, corrected_dq0) = self.flex.correctDeflections(
            left_flexing_torque,
            right_flexing_torque,
            self.encoders_q.copy(),
            self.encoders_dq.copy(),
        )
        (corrected_q1, corrected_dq1) = self.flex.correctDeflections(
            left_flexing_torque,
            right_flexing_torque,
            self.encoders_q.copy(),
            self.encoders_dq.copy(),
        )

        self.assertTrue(not (corrected_q0 == corrected_q1).all())
        self.assertTrue(not (corrected_dq0 == corrected_dq1).all())

        self.flex.reset()
        (corrected_q3, corrected_dq3) = self.flex.correctDeflections(
            left_flexing_torque,
            right_flexing_torque,
            self.encoders_q.copy(),
            self.encoders_dq.copy(),
        )

        self.assertTrue((corrected_q0 == corrected_q3).all())
        self.assertTrue((corrected_dq0 == corrected_dq3).all())


if __name__ == "__main__":
    unittest.main()
