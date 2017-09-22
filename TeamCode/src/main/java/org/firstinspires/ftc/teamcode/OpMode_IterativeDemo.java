/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**jewfnejfbwifbi
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 */


@TeleOp(name="Iterative OpMode Template", group="Teleop A-Team")  // @Autonomous(...) is the other common choice
public class OpMode_IterativeDemo extends OpMode{
    public int driveMode;
public double maxSpeed;
    public double velocity;




    /* Declare OpMode members. */

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {

        driveMode = 0;
        maxSpeed = 0.5;
        velocity =  0.0;
 //gamepad 1-2 gamepad 2-3  max starts at 0.5
        //in init initialize max speed
               // gampepad 1--decrase by 0.01
               // gamepad b-increases by 0.07
                //optional: dpad.down-decreases drive mode to 2 and increases speed to 0.095

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAy
     */


    @Override
    public void loop() {
        if (gamepad1.a){
        driveMode = 3;
            maxSpeed = 0.5;
        }
        if (gamepad1.b){
        driveMode = 2;
            maxSpeed = 0.5;
        }

        if (maxSpeed>0.5){
        maxSpeed=maxSpeed-0.01;
        }

        if (driveMode == 3){
            velocity += 0.03;
            if (velocity > maxSpeed) {
                velocity = maxSpeed;
            }
        }

        if (driveMode == 2){
            velocity += 0.01;
            if (velocity > maxSpeed) {
                velocity = maxSpeed;
            }
        }




                    }






        // gampepad 1--decrase by 0.01
        // gamepad b-increases by 0.07

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
