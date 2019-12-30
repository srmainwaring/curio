//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2019 Rhys Mainwaring
//  All rights reserved
//   
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1.  Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//  2.  Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//  3.  Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

/*
 * Author: Rhys Mainwaring
 */

#ifndef ACKERMANN_DRIVE_CONTROLLER_ACKERMANN_DRIVE_ENUMS_H_
#define ACKERMANN_DRIVE_CONTROLLER_ACKERMANN_DRIVE_ENUMS_H_

namespace ackermann_drive_controller
{
    enum AckermannWheelIndex
    {
        WHEEL_INDEX_FRONT_LEFT  = 0,
        WHEEL_INDEX_FRONT_RIGHT = 1,
        WHEEL_INDEX_MID_LEFT    = 2,
        WHEEL_INDEX_MID_RIGHT   = 3,
        WHEEL_INDEX_BACK_LEFT   = 4,
        WHEEL_INDEX_BACK_RIGHT  = 5
    };

    enum AckermannSteerIndex
    {
        STEER_INDEX_FRONT_LEFT  = 0,
        STEER_INDEX_FRONT_RIGHT = 1,
        STEER_INDEX_BACK_LEFT   = 2,
        STEER_INDEX_BACK_RIGHT  = 3
    };

} // namespace ackermann_drive_controller

#endif // ACKERMANN_DRIVE_CONTROLLER_ACKERMANN_DRIVE_ENUMS_H_
