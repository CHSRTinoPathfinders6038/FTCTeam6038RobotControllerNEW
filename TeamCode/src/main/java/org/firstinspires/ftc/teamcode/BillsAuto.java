package org.firstinspires.ftc.teamcode;

/**
 * Created by beta on 10/2/17.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="BillsAuto", group="Team-A")
public class BillsAuto extends LinearOpMode {
    VuforiaLocalizer vuforia;
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private String teamSide = "RED"; // This must be changed during the competition.
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.2;
    static final double     ROBOT_SPEED             = 2.25;
    ColorSensor colorSensor;
    ColorSensor jewelSensor;
    public static final double MID_SERVO = 0.5 ;

    @Override
    public void runOpMode() {
        /**
        robot.leftClaw.setPosition(MID_SERVO);
        if(detectJewel().equals("RED"))
        {
            robot.leftClaw.setPosition(1.0);
        }
        if(detectJewel().equals("BLUE"))
        {
            robot.leftClaw.setPosition(0.0);
        }
         */






        /**
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("cr");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**
         * This is the autonomous code that includes the encoder drive
         * to the columns
         */
        //
        // forwardUntilLine();
        /**
        telemetry.addData("Path", "Complete, Traveled distance 12");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 2, 2, 5.0);
        //
        encoderDrive(TURN_SPEED, -3.9,  3.9, 5.0);
        //5.85 is 90
        //3.9 is 60
        //45 for 3
        encoderDrive(DRIVE_SPEED, 4, 4, 5.0);
        telemetry.addData("Path", "McLeod Is A MEMEME");
        telemetry.update();

        */
        encoderDrive(DRIVE_SPEED, 2, 2, 2.0);
        encoderDrive(DRIVE_SPEED, -3, -3, 2.0);
        encoderDrive(TURN_SPEED, -5.85, 5.85, 3.0);
        encoderDrive(DRIVE_SPEED, 2, 2, 3.0);
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer. parParametersameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia.The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AQAX3E3/////AAAAGdgJXbEfEE46jUtAgvCh+zMUcooC2pw0cQDyryTvAbzTT2bmfa/ICA2USBJPIOiJtcgkSyFwQhTaks3Ndugus5lHtobUBjgZEWrNrK2xn5AaHO0SMhue0doJ27KsgiuZ6izxPwq5ZwFF3ZrceHDR8oQ1rLgnq2wTPb4NjCYEQToHUoGIjGU6htR7ctOjp11zgNFicEu6vC1/jBV2C1lx6TZ9H8G+4Ea9TzH7XIuuQ4aZuUMnHrS8NSdjNpLp8N2Qu/UlNPkP1qgHiKMhllHei/n5NL8dPxS7Gd6vyY6HsK1M3HKTgGtKoRXpfdWSH9UotSVkUFccH3mTmO3+tvDiL8KKpNtVn6vWbHQQJ6BE9O93";
        //Declare the api key
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if ( vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s Cryptobox Column", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
            }
            else {
                telemetry.addData("VuMark", "Pictograph Not Visible");
            }
            //science guy

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                driveToLine();
                encoderDrive(TURN_SPEED, (5.85/3), (-5.85/3), 2.0 );
                break;
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                driveToLine();
                encoderDrive(TURN_SPEED, (-5.85/3), (5.85/3), 2.0 );
                break;
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                driveToLine();
                encoderDrive(TURN_SPEED, 2,  2, 5.0);
                break;
            }
            telemetry.update();


        }



        /**
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark == RelicRecoveryVuMark.RIGHT) {

            encoderDrive(TURN_SPEED, 3.9,  -3.9, 5.0);
        }
        else if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            encoderDrive(TURN_SPEED, -3.9,  3.9, 5.0);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            encoderDrive(TURN_SPEED, 2,  2, 5.0);
        }
        */




        /**
        encoderDrive(DRIVE_SPEED, 48, 48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
        */
        /**
         * This code right here will get us to the line
         */
        //forwardUntilLine();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void driveToLine() {
        encoderDrive(DRIVE_SPEED, -3, -3, 2.0);
        encoderDrive(TURN_SPEED, -5.85, 5.85, 3.0);
        encoderDrive(DRIVE_SPEED, 2, 2, 3.0);
        encoderDrive(DRIVE_SPEED, -5.85, 5.85, 2.0);

    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void turnInPlace(int degrees) {
        int factor = ((degrees < 0) ? -1 : 1) * 1;
        double turnDistance = Math.PI * 1.21 * (degrees/360.0);
        encoderDrive(TURN_SPEED, factor * turnDistance, -factor * turnDistance, turnDistance/ROBOT_SPEED + 0.5);
    }


    public String getColor(ColorSensor sensorColor)
    {
        telemetry.addData("LED", "Off");
        telemetry.addData("Clear", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();

        if(sensorColor.red() > sensorColor.blue()) return "RED";
        else if(sensorColor.red() < sensorColor.blue()) return "BLUE";
        return null;
    }


    public void forwardUntilLine()
    {
        while (getColor(colorSensor) == null)
        {
            robot.leftDrive.setPower(DRIVE_SPEED);
            robot.rightDrive.setPower(DRIVE_SPEED);
        }
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
    }

    public String detectJewel() {
        return getColor(jewelSensor);

    }


}