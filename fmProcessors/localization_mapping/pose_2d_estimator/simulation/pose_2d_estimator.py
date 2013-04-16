#!/usr/bin/env python
#*****************************************************************************
# Pose 2D Estimator Library
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file contains a 2D pose estimator library designed for field robots.

A pose defines the position and orientation of a shape in world space
(or the parent coordinate space). In this implementation the pose describes
the robot lateral (2D) position in a geographical coordinate plane.

The pose is estimated based on sensor inputs from:
	Robot odometry feedback:
		Wheel encoders (typically fused with a gyro)
	Absolute position sensor:
		Real Time Kinematic GPS (RTK-GPS)
	Absolute orientation sensor:
		Attitude and heading reference system (AHRS)

Each sensor input is validated and filtered, and the actual variance is
estimated. This is based on generic and parameterized knowledge about
the field robot and sensor types.

An Extended Kalman Filter (EKF) then uses the robot odometry feedback as
system input (prediction) and the GPS and AHRS information as measurement
inputs (correction).

The library is expected to be ported to C++ for performance optimations
but during development and test everything will be written in Python.

Revision
2013-04-11 KJ First version
"""
# imports

# defines

class pose_2d_estimator():
    def __init__(self):
		self.prevX = 

		self.H_gnss = np.matrix([[1., 0., 0.,],[0., 1., 0.]]) # GNSS observation matrix

	def ekf_system_update (self, delta_dist, delta_angle):
		u = [delta_dist, delta_angle]

        # predicted (A priori) state estimate: X-[t] = X[t-1] + u[t]
        priX = self.f(prevX, u)

        # predicted (A priori) error covariance estimate: P-[t] = P[t-1] + Q
        Q = Q + np.matrix([[dDist*ODODistVar,0.,0.],[0.,dDist*ODODistVar,0.],[0.,0.,dAngle/(2*pi)*ODOAngleVar]])
        Gm = self.G(prevX, u)
        priCov = Gm*prevCov*Gm.T + Q 

        # housekeeping
        kf.append(array(priX)[0]) # add the A priori state estimate to log
        prevX = priX # advance to next update step
        prevCov = priCov

	def ekf_measurement_update__gnss (self):
        # Compute Kalman gain: K[t] = P-[t]/(P-[t] + R)
        K = prevCov*Hgps.T*linalg.inv(Hgps*prevCov*Hgps.T + R(GPSVar))

        # updated (A posteriori) estimate with measurement z[t]: X[t] = X-[t] + K[t]*(z[t] - X-[t])
        gpsEN = [gpsSim[-1][GPS_E], gpsSim[-1][GPS_N]] # measurement
        prevEN = [array(prevX)[0][0], array(prevX)[0][1]] # A priori position
        y = np.matrix(gpsEN).T - np.matrix(prevEN).T # measurement residual

        postX = prevX + (K*y).T # updated state estimate

        # updated (A posteriori) error covariance: P[t] = (1 - K[t])*P-[t]
        postCov = (mat(np.identity(3))-K*Hgps)*prevCov

        # housekeeping
        Q = np.identity(3) # reset the process noise covariance matrix
        kf.append(array(postX)[0]) # add the A posteriori state estimate to log
        prevX = postX # advance to next update step
        prevCov = postCov # save A posteriori error variance estimate

	def ekf_measurement_update_ahrs (self):
		pass

	def f (x, u): # predict state
		# x is the state vector at timestep k: [X, Y, theta].T
		# u is the system input vector at timestep k: [delta_dist, delta_angle].T
		x = array(x)[0]
		theta = x[2]
		dTheta = u[1]
		l1 = [x[0]+u[0]*cos(theta+dTheta/2.),x[1]+u[0]*sin(theta+dTheta/2.),theta+dTheta]
		return np.matrix(l1)

	def G (x, u): # system error covariance matrix
		x = array(x)[0]
		theta = x[2]
		dTheta = u[1]
		l1 = [1., 0., -u[0]*sin(theta+dTheta/2.)]
		l2 = [0., 1., u[0]*cos(theta+dTheta/2.)]
		l3 = [0., 0., 1.]
		return (np.matrix([l1, l2, l3]))


