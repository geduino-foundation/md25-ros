/*
 pid.cpp

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

#include "pid.h"

PID::PID(double _kLinear, double _kdLinear, double _kAngular, double _kdAngular) :
    kLinear(_kLinear),
    kdLinear(_kdLinear),
    kAngular(_kAngular),
    kdAngular(_kdAngular) {

    // Set initial speeds
    speed1 = 128;
    speed2 = 128;

    // Set initialized to false
    initialized = false;

}

void PID::update(double dLinear, double dAngular, double dTime) {

    // Update speeds with direct gain
    speed1 += dLinear * kLinear - dAngular * kAngular;
    speed2 += dLinear * kLinear + dAngular * kAngular;

    if (initialized) {

        // Compute derivates
        double derDlinear = (dLinear - lastDlinear) / dTime;
        double derDangular = (dAngular - lastDangular) / dTime;

        // Update speed with derivative gain
        speed1 += derDlinear * kdLinear - derDangular * kdAngular;
        speed2 += derDlinear * kdLinear + derDangular * kdAngular;

    } else {

        // Set initialized to true
        initialized = true;

    }

    // Apply constraint
    constrainSpeed(speed1);
    constrainSpeed(speed2);

    // Update last deltas
    lastDlinear = dLinear;
    lastDangular = dAngular;

}

void PID::set(double _kLinear, double _kdLinear, double _kAngular, double _kdAngular) {

    // Set gains
    kLinear = _kLinear;
    kdLinear = _kdLinear;
    kAngular = _kAngular;
    kdAngular = _kdAngular;

}

void PID::getSpeeds(int * _speed1, int * _speed2) {

    // Get speeds
    * _speed1 = (int) speed1;
    * _speed2 = (int) speed2;

}

void PID::constrainSpeed(double & _speed) {

    if (_speed > 255) {

        // Force upper limit
        _speed = 255;

    } else if (_speed < 0) {

        // Force lower limit
        _speed = 0;

    }

}
