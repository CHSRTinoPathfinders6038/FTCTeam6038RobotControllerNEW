package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by billx on 11/13/17.
 */

    @Autonomous(name="BillsAutoGyro", group="Team-A")
    public class BillsAutoGyro extends LinearOpMode {
        VuforiaLocalizer vuforia;
        HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
        ModernRoboticsI2cGyro gyro    = null;
        private ElapsedTime runtime = new ElapsedTime();
        private String teamSide = "RED"; // This must be changed during the competition.
        static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        static final double DRIVE_SPEED = 0.3;
        static final double TURN_SPEED = 0.2;

        static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
        static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

        static final double     ROBOT_SPEED             = 2.25;
        ColorSensor colorSensor;
        ColorSensor jewelSensor;
        public static final double MID_SERVO = 0.5 ;

        @Override
        public void runOpMode() {


            /**
             /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);
            colorSensor = hardwareMap.colorSensor.get("cr");
            jewelSensor = hardwareMap.colorSensor.get("js");
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData(">", "Calibrating Gyro");    //
            telemetry.update();

            gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (!isStopRequested() && gyro.isCalibrating())  {
                sleep(50);
                idle();
            }

            telemetry.addData(">", "Robot Ready.");    //
            telemetry.update();

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)

            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
            }

            gyro.resetZAxisIntegrator();

            waitForStart();

            gyroDrive(DRIVE_SPEED, 48.0, 0.0);
//            gyroTurn( TURN_SPEED, -45.0);
//            gyroHold( TURN_SPEED, -45.0, 0.5);
//            gyroDrive(DRIVE_SPEED, 12.0, -45.0);
//            gyroTurn( TURN_SPEED,  45.0);
//            gyroHold( TURN_SPEED,  45.0, 0.5);
//            gyroTurn( TURN_SPEED,   0.0);
//            gyroHold( TURN_SPEED,   0.0, 1.0);
//            gyroDrive(DRIVE_SPEED,-48.0, 0.0);

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
//            encoderDrive(DRIVE_SPEED, 2, 2, 2.0);
//            encoderDrive(DRIVE_SPEED, -3, -3, 2.0);
//            encoderDrive(TURN_SPEED, -5.85, 5.85, 3.0);
//            encoderDrive(DRIVE_SPEED, 2, 2, 3.0);
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
//                    encoderDrive(TURN_SPEED, (5.85/3), (-5.85/3), 2.0 );
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT)
                {
                    driveToLine();
//                    encoderDrive(TURN_SPEED, (-5.85/3), (5.85/3), 2.0 );
                    break;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    driveToLine();
//                    encoderDrive(TURN_SPEED, 2,  2, 5.0);
                    break;
                }
                telemetry.update();


            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }




    String format(OpenGLMatrix transformationMatrix) {
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }

        public void driveToLine() {

        }

        public void turnInPlace(int degrees) {
            int factor = ((degrees < 0) ? -1 : 1) * 1;
            double turnDistance = Math.PI * 1.21 * (degrees/360.0);
//            encoderDrive(TURN_SPEED, factor * turnDistance, -factor * turnDistance, turnDistance/ROBOT_SPEED + 0.5);
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

