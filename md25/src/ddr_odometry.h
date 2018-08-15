/*
 ddr_odometry.h

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

#ifndef DDR_ODOMETRY_H
#define DDR_ODOMETRY_H

#include <math.h>
#include <rolling_window.h>
#include <Eigen/Core>

typedef Eigen::Matrix<double, 3, 1> Vector3;

class DDROdometry {

    public:

        // Create odometry for a DDR with given wheel base
        DDROdometry(char _rollingWindowSize);

        // Update position for given wheel path increments and time
        void update(double ds, double dth, double dt);

        // Get position vector as (x, y, th)
        void getPosition(Vector3 & _pos);

        // Get velocity vector as (vx, vy, vth)
        void getVelocity(Vector3 & _vel);

        // Reset position
        void reset();

    private:

        // The last update time
        double lastUpdateTime;

        // The position and velocity vector
        Vector3 pos, vel;

        // The rolling windows used to compute velocity
        RollingWindow dsRollingWindow, dthRollingWindow, dtRollingWindow;

        // Update position
        void updatePosition(double ds, double dth);

        // Update velocity
        void updateVelocity(double ds, double dth, double dt);

};

#endif
