/*
 asleep_rate.h

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

#ifndef ASLEEPRATE_H
#define ASLEEPRATE_H

class AsleepRate {

    public:

        // Create asleep rate with give frequency
        AsleepRate(double frequency);

        // Check if rate is ellapsed
        bool ellapsed(double time);

    private:

        // The cycle duration
        double duration;

        // The last execution time
        double lastExecutionTime;
};

#endif
