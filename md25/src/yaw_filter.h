/*
 yaw_filter.h

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

#ifndef YAWFILTER_H
#define YAWFILTER_H

class YawFilter {

    public:

        // Create new yaw filter with give tau
        YawFilter(double _tau);

        // Filter raw yaw using given velocity
        void filter(double raw, double velocity, double dTime);

        // Set filter tau
        void set(double _tau);

        // Get filtered yaw
        void getYaw(double & _yaw);

        // Get filtered yaw delta with previous value
        void getDelta(double & _dYaw);

    private:

        // The filter tau
        double tau;

        // The filtered yaw
        double yaw, lastYaw;

};

#endif
