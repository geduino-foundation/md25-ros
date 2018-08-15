/*
 pid.h

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

#ifndef PID_H
#define PID_H

class PID {

    public:

        // Create new PID with given gains
        PID(double _kLinear, double _kdLinear, double _kAngular, double _kdAngular);

        // Update PID with given deltas
        void update(double dLinear, double dAngular, double dTime);

        // Set PID gains
        void set(double _kLinear, double _kdLinear, double _kAngular, double _kdAngular);

        // Get ouput speeds
        void getSpeeds(int * _speed1, int * _speed2);

    private:

        // Contrain speed in [0, 255] range
        void constrainSpeed(double & _speed);

        // The PID gain
        double kLinear, kdLinear, kAngular, kdAngular;

        // The last delta
        double lastDlinear, lastDangular;

        // Delta initialized flag
        bool initialized;

        // The output
        double speed1, speed2;

};

#endif
