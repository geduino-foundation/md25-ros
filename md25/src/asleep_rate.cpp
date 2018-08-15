/*
 asleep_rate.cpp

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

#include "asleep_rate.h"

AsleepRate::AsleepRate(double frequency) :
    duration(1.0 / frequency),
    lastExecutionTime(0) {
}

bool AsleepRate::ellapsed(double time) {

    if (lastExecutionTime + duration > time) {

        // Set last execution time
        lastExecutionTime = time;

        return true;

    } else {
        return false;
    }

}
