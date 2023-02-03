#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 13:47:16 2022

@author: nvilla
"""

import unittest
import numpy as np
from flex_joints import Flex


class FlexTestCase(unittest.TestCase):
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
            flexToJoint=flexToJoint,
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
        self.H_stiff = H_stiff
        self.H_damp = H_damp

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

    def test_flexing_torque(self):

        T = (np.random.rand() * 2 - 1) * 80
        t = (np.random.rand() * 2 - 1) * 0.4
        tau_0 = np.zeros(3)
        tau_z = np.array([T, 0, 0])
        tau_x = np.array([0, T, 0])
        tau_y = np.array([0, 0, T])

        hip_0 = np.zeros(3)
        hip_z = np.array([t, 0, 0])
        hip_x = np.array([0, t, 0])
        hip_y = np.array([0, 0, t])

        flexTau = self.flex.estimateFlexingTorque(hip_0, tau_0)
        self.assertTrue((flexTau == [0, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_0, tau_z)
        self.assertTrue((flexTau == [0, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_0, tau_x)
        self.assertTrue((flexTau == [0, T]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_0, tau_y)
        self.assertTrue((flexTau == [T, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_z, tau_z)
        self.assertTrue((flexTau == [0, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_x, tau_z)
        self.assertTrue((flexTau == [0, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_y, tau_z)
        self.assertTrue((flexTau == [0, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_z, tau_x)
        self.assertTrue((np.abs(flexTau) <= np.abs([T, T])).all())

        flexTau = self.flex.estimateFlexingTorque(hip_z, tau_y)
        self.assertTrue(
            (np.abs(flexTau) < np.abs([T, T])).all()
            and (np.abs(flexTau) > [0, 0]).all()
        )

        flexTau = self.flex.estimateFlexingTorque(hip_y, tau_y)
        self.assertTrue((flexTau == [T, 0]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_x, tau_x)
        self.assertTrue((flexTau == [0, T]).all())

        flexTau = self.flex.estimateFlexingTorque(hip_x, tau_y)
        self.assertTrue((np.abs(flexTau) <= np.abs([T, 0])).all())

        flexTau = self.flex.estimateFlexingTorque(hip_y, tau_x)
        self.assertTrue((flexTau == [0, T]).all())

        delta = np.array([0, 0])
        dt = 0.002
        for i in range(1000):
            delta = self.flex.computeDeflection(
                tau_x[:2], delta, self.H_stiff[:2], self.H_damp[:2], dt
            )

        force_z = np.array([0, 0, np.abs(10 * T)])
        flexTau = self.flex.estimateFlexingTorque(
            np.hstack([0, -delta[1], -delta[0]]), tau_x
        ).copy()
        flexTau_force_p = self.flex.estimateFlexingTorque(
            np.hstack([0, -delta[1], -delta[0]]), tau_x, delta, force_z
        ).copy()
        flexTau_force_m = self.flex.estimateFlexingTorque(
            np.hstack([0, -delta[1], -delta[0]]), tau_x, delta, -force_z
        ).copy()

        self.assertTrue((np.abs(flexTau) <= np.abs(flexTau_force_p)).all())
        self.assertTrue((np.abs(flexTau) >= np.abs(flexTau_force_m)).all())

        # Note: as expected, positive vertical forces (supporting hip) tend to
        # increase the magniture of the flexing torque, while negative verical
        # forces (swinging hip) tends to reduce it.

    def test_current_flex_to_joint(self):

        delta = np.random.rand(2) - 0.5

        original = self.flex.Settings()["flexToJoint"]
        originalComputed = self.flex.currentFlexToJoint(np.array([0, 0])).copy()
        current = self.flex.currentFlexToJoint(delta)

        self.assertTrue(
            (np.linalg.norm(original) - np.linalg.norm(current) < 1e-10).all()
        )
        self.assertTrue((original == originalComputed).all())

    def test_rotations(self):

        q0 = self.encoders_q
        dq0 = self.encoders_dq
        desired_torques = np.zeros(12)

        q, dq = self.flex.correctEstimatedDeflections(desired_torques, q0, dq0)

        q1 = q0.copy()
        for o in np.linspace(0, np.pi / 2, 20):

            q1[0] = o
            q1[6] = o
            q, dq = self.flex.correctEstimatedDeflections(desired_torques, q1, dq0)

            self.assertTrue(q[0] - q[6] < 1e-13)
            self.assertTrue(q[0] - o < 1e-13)


if __name__ == "__main__":
    unittest.main()
