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
 * This OpMode
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpv1.0", group = "Team-A")
public class BillsTeleOp extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private int driveMode;
    private double maxSpeed;

    public void init() {

        driveMode = 1;
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
//      rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        maxSpeed = 1;

//      buttonPress.setPosition(0.25);

    }

    public void loop() {


        float leftY = -gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x;
        float rightX = gamepad1.right_stick_x;
        float rightY = -gamepad1.right_stick_y;

        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        if (gamepad1.a) {
            telemetry.addData("Drive Mode", "Tank Drive Test");
            driveMode = 3;
        }
        if (gamepad1.b) {
            driveMode = 4;
            telemetry.addData("Drive Mod    ze", "Bills Premier Arcade AllRounder");
            telemetry.update();
        }
        if (gamepad1.x) {
            telemetry.addData("Drive Mode", "Arcade Forward");
            driveMode = 1;
        }
        if (gamepad1.y) {
            driveMode = 2;
            telemetry.addData("Drive Mode", "Arcade Backward");
            telemetry.update();
        }

//      For test purposes only - may remove in the future
        if (driveMode == 1) {
            leftMotor.setPower((leftY + leftX) * maxSpeed);
            rightMotor.setPower((leftY - leftX) * maxSpeed);
        }

        if (driveMode == 3) {
            leftMotor.setPower(y * maxSpeed);
            rightMotor.setPower(y * maxSpeed);
        }

//      For test purposes only - may remove in the future
        if (driveMode == 2) {
            leftMotor.setPower(-1 * (leftY + leftX) * maxSpeed);
            rightMotor.setPower(-1 * (leftY - leftX) * maxSpeed);
        }
        if (driveMode == 4) {
            double scaledRight = scaleInput((y - x) / Math.sqrt(2.0));
            double scaledLeft = scaleInput((y + x) / Math.sqrt(2.0));

            leftMotor.setPower(scaledLeft / 2);
            rightMotor.setPower(scaledRight / 2);
        }

        if (gamepad1.dpad_down) {
            maxSpeed -= 0.01;
            if (maxSpeed < 0) {
                maxSpeed = 0;
            }
        } else if (gamepad1.dpad_up) {
            maxSpeed += 0.01;
            if (maxSpeed > 1) {
                maxSpeed = 1;
            }
        }

        telemetry.addData("Gamepad 1", "");
        telemetry.addData("1", "Tank: Left Stick to move leftMotor side, Right Stick to move rightMotor side");
        telemetry.addData("2", "D-Pad down to decrease max speed, D-Pad up to increase up to 100%");
        telemetry.addData("3", "Press B to switch to Bills, Press A to switch back to forward tank driving");
        telemetry.addData("4", "Press X to switch to forward arcade driving, Press Y to switch to backward arcade driving");
        telemetry.addData("Gamepad 2", "");
        telemetry.addData("1 ", "Future Specialized Functions go here!");
        telemetry.addData("Current Max Speed", maxSpeed * 100 + "%");
        telemetry.update();
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

