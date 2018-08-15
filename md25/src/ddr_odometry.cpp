/*
 ddr_odometry.cpp

 Copyright (C) 2018 Alessandro Francescon

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "md25_odometry.h"

#define TWO_PI 6.28318530717959

DDROdometry::DDROdometry(char _rollingWindowSize) :
      dsRollingWindow(RollingWindow(_rollingWindowSize)),
      dthRollingWindow(RollingWindow(_rollingWindowSize)),
      dtRollingWindow(RollingWindow(_rollingWindowSize)) {

    reset();

}

void DDROdometry::update(double ds, double dth, double dt) {

    // Update position
    updatePosition(ds, dth);

    // Update velocity
    updateVelocity(ds, dth, dt);

}

void DDROdometry::getPosition(Vector3 & _pos) {
    _pos = pos;
}

void DDROdometry::getVelocity(Vector3 & _vel) {
    _vel = vel;
}

void DDROdometry::reset() {

   // Reset last update time
   lastUpdateTime = 0;

   // Reset position and velocity
   pos.fill(0);
   vel.fill(0);

   // Reset rolling window
   dsRollingWindow.reset();
   dthRollingWindow.reset();
   dtRollingWindow.reset();

}

void DDROdometry::updatePosition(double ds, double dth) {

    // Calculate middle theta
    double mdth = pos(2) + dth * 0.5;
    double sinmdth = sin(mdth);
    double cosmdth = cos(mdth);

    // Update position
    pos(0) += ds * cosmdth;
    pos(1) += ds * sinmdth;
    pos(2) += dth;

    // Make sure orientation is between [0, 2PI[
    while (pos(2) >= TWO_PI) {
        pos(2) -= TWO_PI;
    }
    while (pos(2) < 0) {
        pos(2) += TWO_PI;
    }

}

void DDROdometry::updateVelocity(double ds, double dth, double dt) {

    // Add values to rolling window
    dsRollingWindow.add(ds);
    dthRollingWindow.add(dth);
    dtRollingWindow.add(dt);

    // Get sums
    double dsSum;
    double dthSum;
    double dtSum;
    dsRollingWindow.sum(& dsSum);
    dthRollingWindow.sum(& dthSum);
    dtRollingWindow.sum(& dtSum);

    if (dtSum > 0) {

        // Update velocity
        vel(0) = dsSum / dtSum;
        vel(2) = dthSum / dtSum;

    } else {

        // Set linear and angular velocity to zero (avoid by zero division)
        vel(0) = 0;
        vel(2) = 0;

    }

}
