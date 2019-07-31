#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2019 Bruno Tiberio
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# This code implements the algorithm presented by Sebastian Madgwick with changes
# to fit the changes described particular axis and specific application.
#
# Original documentation can be seen in:
#   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
#
#   Date          Author          Notes
#   28/09/2011    SOH Madgwick    Initial release
#   25/03/2017    Bruno Tibério   Updated for use frame axis represented in
#                                 razor9dof (v10125).
#                                 Added functions for performing updates
#                                 based only on magnetometer and/or
#                                 gyroscopes. Changed algorithm based on
#                                 calibration limitation
#   15/05/2019    Bruno Tibério   Moving from Matlab implementation to python
#
# --------------------------------------------------------------------------
#
# Notes: This algorithm is heavily modified to fit the axis convention used
# in the Sparkfun Razor9DOF (v10125). Also the algorithm is adapted for
# being used inside a car (a real car, not a toy or robot). This causes
# impossibility of magnetometer 3D calibration.
# Please refer to
# https://fenix.tecnico.ulisboa.pt/cursos/meec/dissertacao/1409728525631969
# for more details.
#
# --------------------------------------------------------------------------



import numpy as np
import quaternion
from numpy.linalg import norm
import logging
import sys


class Madgwick:

    _sample_period = 1/256
    _quaternion = np.quaternion(1, 0, 0, 0)
    beta = 1
    logger = None
    _debug = False
    _gravity = np.quaternion(0, 0, 0, 1)

    def __init__(self, debug=False, sample_period=1/256.0):
        self.logger = logging.getLogger("Madgwick")
        if debug:
            self._debug = debug
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)
        self.sample_period = sample_period

    def update(self):
        pass

    def update_imu(self):
        pass

    def update_gyro(self):
        pass

    def update_mag(self):
        pass

    def _set_sample_period(self, value):
        if not np.isreal(value):
            raise ValueError("Value must be real number.")
        self._sample_period = value
        return

    def _get_sample_period(self):
        return self._sample_period

    def _set_quaternion(self, q):
        if not isinstance(q, np.quaternion):
            if len(q) != 4:
                raise TypeError("Input must be a quaternions type  or a 4-element array")
            self._quaternion = np.quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        else:
            self._quaternion = q
        return

    def _get_quaternion(self):
        return self._quaternion

    def log_info(self, message=None):
        """ Log a message
        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message
        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return
        self.logger.info('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def log_debug(self, message=None):
        """ Log a message with debug level
        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message
        the function name will be the caller function retrieved automatically
        by using sys._getframe(1).f_code.co_name
        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return
        self.logger.debug('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    # for easy handles
    sample_period = property(fset=_set_sample_period, fget=_get_sample_period)
    q = property(fset=_set_quaternion, fget=_get_quaternion)

