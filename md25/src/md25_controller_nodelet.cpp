/*
 md25_controller_nodelet.cpp

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

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "md25_controller.h"

namespace md25 {

    class MD25ControllerNodelet : public nodelet::Nodelet {

        public:

            MD25ControllerNodelet() {};

        private:

            virtual void onInit() {

                // Get node handle
                ros::NodeHandle nodeHandle = getNodeHandle();
                ros::NodeHandle privateNodeHanlde = getPrivateNodeHandle();

                // Create md25 controller
                md25Controller.reset(new MD25Controller(nodeHandle, privateNodeHanlde));

                // Init md25 controller
                md25Controller->init();

            }

            boost::shared_ptr<MD25Controller> md25Controller;

    };

}

PLUGINLIB_DECLARE_CLASS(md25, MD25ControllerNodelet, md25::MD25ControllerNodelet, nodelet::Nodelet);


