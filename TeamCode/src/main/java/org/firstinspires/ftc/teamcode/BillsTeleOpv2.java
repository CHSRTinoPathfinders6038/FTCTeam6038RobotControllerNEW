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
 * Created by beta on 9/25/17.
 *
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOpv2.0", group = "Team-A")


public class BillsTeleOpv2 extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DriveMode driveMode;
    private double maxSpeed;
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry.Item elapsedTime;

    /*
     *POV Mode uses the left stick to control forward-backward motion,
     *and the right stick to control left-right motion.
     *OPP is reversed.
     */
    private enum DriveMode {
        ARCADE_MODE,
        TANK_MODE,
        POV_MODE,
        OPP_POV_MODE;
    }

    @Override
    public void init() {

        driveMode = DriveMode.ARCADE_MODE;

        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        maxSpeed = 1;

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Gamepad 1", "");
        telemetry.addData("1", "Tank: Left Stick to move leftMotor side, Right Stick to move rightMotor side");
        telemetry.addData("2", "D-Pad down to decrease max speed, D-Pad up to increase up to 100%");
        telemetry.addData("3", "Press B to switch to Bills, Press A to switch back to forward tank driving");
        telemetry.addData("4", "Press X to switch to forward arcade driving, Press Y to switch to backward arcade driving");
        telemetry.addData("Gamepad 2", "");
        telemetry.addData("1 ", "Future Specialized Functions go here!");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        elapsedTime = telemetry.addData("Time:", "", runtime.toString());

        telemetry.update();
    }

    public void start() {
        telemetry.clear();
        runtime.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            telemetry.addData("Drive Mode", "Arcade");
            driveMode = DriveMode.ARCADE_MODE;
            telemetry.update();
        }
        if (gamepad1.b) {
            telemetry.addData("Drive Mode", "Tank");
            driveMode = DriveMode.TANK_MODE;
            telemetry.update();
        }
        if (gamepad1.x) {
            telemetry.addData("Drive Mode", "POV-1");
            driveMode = DriveMode.POV_MODE;
            telemetry.update();
        }
        if (gamepad1.y) {
            telemetry.addData("Drive Mode", "POV-2");
            driveMode = DriveMode.OPP_POV_MODE;
            telemetry.update();
        }

        switch (driveMode) {
            case ARCADE_MODE:
                // May need to negate both x and y based on if values do not apply to the
                //REV Robotics HD Hex Motor
                double x = gamepad1.right_stick_x;
                double y = -gamepad1.right_stick_y;

                double scaledRight = scaleInput((y - x) / Math.sqrt(2.0));
                double scaledLeft = scaleInput((y + x) / Math.sqrt(2.0));

                leftMotor.setPower((scaledLeft) * maxSpeed);
                rightMotor.setPower((scaledRight) * maxSpeed);
                break;
            case TANK_MODE:
                double ly = -gamepad1.left_stick_y;
                double ry = -gamepad1.right_stick_y;

                leftMotor.setPower(ly * maxSpeed);
                rightMotor.setPower(ry * maxSpeed);
                break;
            case POV_MODE:

                double leftPower;
                double rightPower;

                // May need negate the values, and respectively for the OPP_POV mode.

                double drive = -gamepad1.left_stick_y;
                double turn  =  gamepad1.right_stick_x;

                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

                leftMotor.setPower((leftPower) * maxSpeed );
                rightMotor.setPower((rightPower) * maxSpeed);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

                break;
            case OPP_POV_MODE:

                drive = -gamepad1.right_stick_y;
                turn  =  gamepad1.left_stick_x;

                leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower = Range.clip(drive - turn, -1.0, 1.0) ;


                leftMotor.setPower((leftPower) * maxSpeed);
                rightMotor.setPower((rightPower) * maxSpeed);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                break;
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

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Current Max Speed", maxSpeed * 100 + "%");
        telemetry.update();

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
