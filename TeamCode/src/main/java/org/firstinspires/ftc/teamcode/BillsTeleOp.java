/* Copyright (c) 2017 Bill Xiang. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Bill nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

/**
 * Created by billx on 9/14/17.
 * This OpMode provides an excellent demonstration to illustrate the means to which
 * a typical TeleOp may be programmed for a bare-bone, four-wheel starter robot will run.
 * It is complete with the following abilities as of at the time of writing:
 *
 * 1) Use buttons on the gamepad to switch the mode of drive (e.g. Arcade, Tank).
 * 2) Adjust the max speed of the motors.
 * 3) Includes a telemetry of instructions for the driver.
 *
 * The code is structured as an IterativeOpMode.
 *
 *{@link org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative}
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**The below @TeleOp annotation specifies that the name of the OpMode shall be known as "TeleOpv1.0"
 * and that the group to which it belongs which be "Team-A". In the case that you create a
 * driver-controlled OpMode, you shall include the following annotation, complete with the
 * appropriate name for the OpMode, as well as specifying the group to be of "Team-B".
 * Additionally, to complete an official registration of an OpMode in general through the system,
 * there is a registering Java class apt to account that for that matter.
 *
 * @see RegisterOpModes
 *
 */

@TeleOp(name = "TeleOpv1.0", group = "Team-A")

public class BillsTeleOp extends OpMode {
    private DcMotor leftMotor; //initializes the left dc motor as DcMotor
    private DcMotor rightMotor;//initializes the right dc motor  as DcMotor
    private int driveMode;   //initializes the drivemode as integer
    private double maxSpeed;  //initializes the maxSpeed as a double

    public void init() { /* this loop sets driveMode to 1 and max speed to 1. It assignes the
    variable leftMotor to the left motor in the hardware map. It assignes the variable rightMotor to
    the right motor in the hardware map.
*/
        driveMode = 1;
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");



        maxSpeed = 1;



    }
    /* This loop sets the variables for the right and left joystick and changes the driveMode when the
    a b x and y to tank, bills arcade, forwards arcade, or backwards arcade and sets the motors
    power acordingly to the driveMode. Also it sets the left and right motors direction. It makes it
    so that the speed is gradualy decreased if the down is pressed on the d-pad and speed increases
    if up is pressed on the d-pad.
    */
    public void loop() { /* This loop sets the variables for the right and left joystick and changes
     the driveMode when the a b x and y to tank, bills arcade, forwards arcade, or backwards arcade
     and sets the motors power acordingly to the driveMode. Also it sets the left and right motors
     direction. It makes it so that the speed is gradualy decreased if the down is pressed on the
     d-pad and speed increases if up is pressed on the d-pad.

*/
        float leftY = -gamepad1.left_stick_y; //sets leftY to the negative y position of the left stick
        float leftX = gamepad1.left_stick_x;   //sets leftX to the x position of the left stick


        double x = gamepad1.right_stick_x; //assigns the position of the right stick x to the variable x
        double y = gamepad1.right_stick_y;   //assigns the position of the right stick y to the variable y

        leftMotor.setDirection(DcMotor.Direction.REVERSE);  //sets the direction of the leftMotor to reverse so that the robot can move forward
        rightMotor.setDirection(DcMotor.Direction.FORWARD);    //sets the direction of the rightMotor to forward so that the robot can move fora

        if (gamepad1.a) { //If the a button is pressed it sets the driveMode to tank
            telemetry.addData("Drive Mode", "Tank Drive Test");  //displays on phone "Tank Drive Test"
            driveMode = 3;  //sets driveMode to tank(3)
        }
        if (gamepad1.b) { //If the b button is pressed it sets the driveMode to Bills arcade allrounder
            driveMode = 4;  //sets driveMode to Arcade(4)
            telemetry.addData("Drive Mode", "Bills Premier Arcade AllRounder");  //displays on phone "Bills Premier Arcade AllRounder"
            telemetry.update();
        }

        if (gamepad1.x) { //If x button is pressed it sets the driveMode to Arcade forwards
            telemetry.addData("Drive Mode", "Arcade Forward"); // Displays on phone "Arcade Forward"
            driveMode = 1; // sets driveMode to Arcade Forward(1)
        }
        if (gamepad1.y) { //If y button is pressed it sets driveMode to arcade backward
            driveMode = 2; // sets driveMode to arcade backwards(2)
            telemetry.addData("Drive Mode", "Arcade Backward"); // displayes on phone "Arcade Backwards"
            telemetry.update();
        }

        if (driveMode == 1) {
            leftMotor.setPower((leftY + leftX) * maxSpeed);   //if driveMode is equal to 1 set the power of leftMotor to thing
            rightMotor.setPower((leftY - leftX) * maxSpeed);  //set power of rightMotor to thing
        }

        if (driveMode == 3) {
            leftMotor.setPower(y * maxSpeed);  //if drivemode is equal to 3 set power of leftMotor to thing
            rightMotor.setPower(y * maxSpeed);    //set power of rightmotor to thing
        }


        if (driveMode == 2) {
            leftMotor.setPower(-1 * (leftY + leftX) * maxSpeed);     //if drivemode is equal to 2 set power of leftmotor to thing
            rightMotor.setPower(-1 * (leftY - leftX) * maxSpeed);        //set poscled to wer of rightmotor to thing
        }
        if (driveMode == 4) {
            double scaledRight = scaleInput((y - x) / Math.sqrt(2.0)); //if drivemode is equal to 4
            double scaledLeft = scaleInput((y + x) / Math.sqrt(2.0));

            leftMotor.setPower(scaledLeft / 2); // set the leftMotor power to scaled left/2
            rightMotor.setPower(scaledRight / 2);// set right motor power to scaled right / 2
        }

        if (gamepad1.dpad_down) { // if you press downkey on dpad
            maxSpeed -= 0.01; // maxpeed = maxspeed - 0.01
            if (maxSpeed < 0) { // if maxspeed is less than zero, set it to zero
                maxSpeed = 0;
            }
        } else if (gamepad1.dpad_up) { // if dpad is up
            maxSpeed += 0.01; // maxspeed +0.01
            if (maxSpeed > 1) { // if maxspeed is greater than 1
                maxSpeed = 1; // set maxspeed to 1 (maxspeeds maximum is 1)
            }
        }

        telemetry.addData("Gamepad 1", ""); // show on phone GAMEPAD 1
        telemetry.addData("1", "Tank: Left Stick to move leftMotor side, Right Stick to move rightMotor side");
        telemetry.addData("2", "D-Pad down to decrease max speed, D-Pad up to increase up to 100%");
        telemetry.addData("3", "Press B to switch to Bills, Press A to switch back to forward tank driving");
        telemetry.addData("4", "Press X to switch to forward arcade driving, Press Y to switch to backward arcade driving");
        telemetry.addData("Gamepad 2", "");
        telemetry.addData("1 ", "Future Specialized Functions go here!");
        telemetry.addData("Current Max Speed", maxSpeed * 100 + "%");
        telemetry.update(); // update telemetry
    }



    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }


        // get value from the array.
        double dScale = 0.0;

        if (dVal < 0) {
            dScale = scaleArray[index];
        } else {
            dScale = -scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


}

