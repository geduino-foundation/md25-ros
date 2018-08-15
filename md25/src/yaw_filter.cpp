/*
 yaw_filter.cpp

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

#include "yaw_filter.h"

#include <iostream>

YawFilter::YawFilter(double _tau) :
    tau(_tau) {
}

double YawFilter::filter(double raw, double velocity, double dTime, double filteredYaw) {

    // Compute filter parameter for current cycle
    double a = tau / (tau + dTime);

    // Filter yaw
    double updatedFilteredYaw = a * (filteredYaw + velocity * dTime) + (1 - a) * raw;

    return updatedFilteredYaw - filteredYaw;

}

void YawFilter::set(double _tau) {
    tau + _tau;
}
