package org.firstinspires.ftc.teamcode

/**
 * Created by beta on 10/2/17.
 * NO ONE CAN TOUCH THIS OR YOU WILL GET A STRIKE FROM BILL INCLUDING OTHER MENTORS
 * IF YOU DO I CAN TRACE THROUGH THE HISTORY AS AN ADMIN AND I WILL FIND YOU
 */


import android.os.UserManager

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

import android.app.ActivityManager
import android.widget.Toast
import org.firstinspires.ftc.teamcode.BillsAuto.MID_SERVO

@Autonomous(name = "BillsAutoLin", group = "Team-A")
class BillsAutoLin : LinearOpMode() {
    internal lateinit var vuforia: VuforiaLocalizer
    internal var robot = HardwarePushbot()   // Use a Pushbot's hardware
    private val runtime = ElapsedTime()

    override fun runOpMode() {
        robot.leftClaw.position = MID_SERVO
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = "AQAX3E3/////AAAAGdgJXbEfEE46jUtAgvCh+zMUcooC2pw0cQDyryTvAbzTT2bmfa/ICA2USBJPIOiJtcgkSyFwQhTaks3Ndugus5lHtobUBjgZEWrNrK2xn5AaHO0SMhue0doJ27KsgiuZ6izxPwq5ZwFF3ZrceHDR8oQ1rLgnq2wTPb4NjCYEQToHUoGIjGU6htR7ctOjp11zgNFicEu6vC1/jBV2C1lx6TZ9H8G+4Ea9TzH7XIuuQ4aZuUMnHrS8NSdjNpLp8N2Qu/UlNPkP1qgHiKMhllHei/n5NL8dPxS7Gd6vyY6HsK1M3HKTgGtKoRXpfdWSH9UotSVkUFccH3mTmO3+tvDiL8KKpNtVn6vWbHQQJ6BE9O93"
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters)
        val relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark")
        val relicTemplate = relicTrackables[0]
        relicTemplate.name = "relicVuMarkTemplate" // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start")
        telemetry.update()
        waitForStart()

        relicTrackables.activate()
        val vuMark = RelicRecoveryVuMark.from(relicTemplate)
        while (vuMark != RelicRecoveryVuMark.UNKNOWN) {


            telemetry.addData("VuMark", "%s Cryptobox Column", vuMark)
        }


        robot.init(hardwareMap)

// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders")    //
        telemetry.update()

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER)

// Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition())
        telemetry.update()

// Wait for the game to start (driver presses PLAY)
        waitForStart()

// Step through each leg of the path,
// Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**
         * This is the autonomous code that includes the encoder drive
         * to the columns
         */
        encoderDrive(DRIVE_SPEED, 48.0, 48.0, 5.0)
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            encoderDrive(TURN_SPEED, 4.0, -4.0, 1.0)// turn Right 30º
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 1.0)//move to the optimal distance
            encoderDrive(TURN_SPEED, -4.0, 4.0, 1.0)//turn left 30º
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 1.0)//move to the optimal distance for the mechanism to put the glphy in the box

        }

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            encoderDrive(TURN_SPEED, -4.0, 4.0, 1.0)// turn left 30º
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 1.0)//move to the optimal distance
            encoderDrive(TURN_SPEED, 4.0, -4.0, 1.0)//turn right 30º
            encoderDrive(DRIVE_SPEED, 4.0, 4.0, 1.0)//move to the optimal distance for the mechanism to put the glphy in the box

        }

        if (vuMark == RelicRecoveryVuMark.CENTER) {
            encoderDrive(DRIVE_SPEED, 12.0, 12.0, 1.0)//move to the optimal distance for the mechanism to put the glphy in the box

        }

        encoderDrive(DRIVE_SPEED, 48.0, 48.0, 5.0)  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, 12.0, -12.0, 4.0)  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 4.0)  // S3: Reverse 24 Inches with 4 Sec timeout


        robot.leftClaw.setPosition(1.0)            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0)
        sleep(1000)     // pause for servos to move

        telemetry.addData("Path", "Complete")
        telemetry.update()
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
    */
    fun encoderDrive(speed: Double,
                     leftInches: Double, rightInches: Double,
                     timeoutS: Double) {
        val newLeftTarget: Int
        val newRightTarget: Int

// Ensure that the opmode is still active
        if (opModeIsActive()) {

// Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (leftInches * COUNTS_PER_INCH).toInt()
            newRightTarget = robot.rightDrive.getCurrentPosition() + (rightInches * COUNTS_PER_INCH).toInt()
            robot.leftDrive.setTargetPosition(newLeftTarget)
            robot.rightDrive.setTargetPosition(newRightTarget)

// Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION)
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION)

// reset the timeout time and start motion.
            runtime.reset()
            robot.leftDrive.setPower(Math.abs(speed))
            robot.rightDrive.setPower(Math.abs(speed))

// keep looping while we are still active, and there is time left, and both motors are running.
// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
// its target position, the motion will stop.  This is "safer" in the event that the robot will
// always end the motion as soon as possible.
// However, if you require that BOTH motors have finished their moves before the robot continues
// onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()))) {

// Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget)
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition())
                telemetry.update()
            }

// Stop all motion;
            robot.leftDrive.setPower(0.0)
            robot.rightDrive.setPower(0.0)

// Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER)

//  sleep(250);   // optional pause after each move
        }
    }

    companion object {

        internal val COUNTS_PER_MOTOR_REV = 1440.0    // eg: TETRIX Motor Encoder
        internal val DRIVE_GEAR_REDUCTION = 2.0     // This is < 1.0 if geared UP
        internal val WHEEL_DIAMETER_INCHES = 4.0     // For figuring circumference
        internal val COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415))
        internal val DRIVE_SPEED = 0.6
        internal val TURN_SPEED = 0.5
        val MID_SERVO = 0.5// set position to 90º
    }
}