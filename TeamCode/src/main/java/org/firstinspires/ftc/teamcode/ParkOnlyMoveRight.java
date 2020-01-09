/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.sql.Driver;


@Autonomous(name="Park Right", group="Linear OpMode")

public class ParkOnlyMoveRight extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    BNO055IMU gyro;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP CHANGE ME IF CRAP GETS WILD
    static final double     WHEEL_DIAMETER_INCHES   = 3.5;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     WHEEL_ANGLE             = Math.PI/4;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.03;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.02;     // Larger is more responsive, but also less stable
    static final double     DRIFT_ADJUST            = 0.75;      // This constant will be used to adjust the speed for
    //  the leftFront and rightRear motors
    static final double     SPEED_INCR              = 0.01;     // This constant is used to ramp-up the speed of the motors

    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    private Servo rightTwist = null;
    private Servo leftTwist = null;

    private Servo rampServo = null;
    private double rampPos = 1;



    @Override
    public void runOpMode()  {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear= hardwareMap.get(DcMotor.class, "rightRear");

        rampServo = hardwareMap.get(Servo.class, "rampServo"); // setting ramp position
        rampServo.setPosition(rampPos);

        rightTwist = hardwareMap.get(Servo.class, "rightTwist");
        leftTwist = hardwareMap.get(Servo.class, "leftTwist");

        rightTwist.setPosition(0); // setting pos of twisters
        leftTwist.setPosition(1);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection((DcMotorSimple.Direction.FORWARD));

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        //gyro.calibrate();
        gyro.initialize(parameters);
        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {

            telemetry.addLine("Waiting to start");
            telemetry.addData("Heading: ", gyro.getAngularOrientation().firstAngle);
            telemetry.update();
        }


        parkRight(0.7, 0.3);


    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double speedIncr,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftRearTarget;
        int     newRightFrontTarget;
        int     newRightRearTarget;
        int     moveCounts;
        double  maxF, maxR, max;
        double  error;
        double  steer;
        double  accelSteer;
        double  leftFrontSpeed;
        double  leftRearSpeed;
        double  rightFrontSpeed;
        double  rightRearSpeed;
        double  rampUpSpeed = speedIncr;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            //  The target positions for the leftFront and rightRear wheels are adjust by the
            //  DRIFT_ADJUST (< 1) as these are the wheels that cause the robot to drift to the right
            moveCounts = (int)(distance * COUNTS_PER_INCH/Math.cos(WHEEL_ANGLE)/2);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(moveCounts * DRIFT_ADJUST);
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(moveCounts * DRIFT_ADJUST);


            // Set Target and Turn On RUN_TO_POSITION
            leftFront.setTargetPosition(newLeftFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            //   The speed for the leftFront and rightRear wheels are adjusted by DRIFT_ADJUST (< 1)
            //   as these are the wheels that cause the robot to drift to the right.
            speed = Range.clip(Math.abs(DRIVE_SPEED /Math.cos(WHEEL_ANGLE)/2), 0.0, 1.0);            // This while loop ensures that the motor speeds are ramped up slowly.  Setting the speeds
            //   suddenly causes the wheels to slip which in turn mess up the distance traveled.
            while (rampUpSpeed < speed) {
                leftFront.setPower(rampUpSpeed * DRIFT_ADJUST);
                leftRear.setPower(rampUpSpeed);
                rightFront.setPower(rampUpSpeed);
                rightRear.setPower(rampUpSpeed * DRIFT_ADJUST);
                rampUpSpeed += SPEED_INCR;
            }


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy()) &&
                    (runtime.seconds() < 30)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
                }

                leftFrontSpeed = speed - steer;
                rightFrontSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
                if (max > 1.0)
                {
                    leftFrontSpeed /= max;
                    rightFrontSpeed /= max;
                }

                leftRearSpeed = leftFrontSpeed;
                rightRearSpeed = rightFrontSpeed;

                // The speed for the leftFront and rightRear wheels are adjusted by DRIFT_ADJUST (< 1)
                //   as these are the wheels that cause the robot to drift to the right.
                leftFront.setPower(leftFrontSpeed*DRIFT_ADJUST);
                leftRear.setPower(leftRearSpeed);
                rightFront.setPower(rightFrontSpeed);
                rightRear.setPower(rightRearSpeed*DRIFT_ADJUST);

            }

            // Stop all motion;
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void gyroStrafe ( double speed,
                             double speedIncr,
                             double distance,
                             double angle) {

        int     newLeftFrontTarget;
        int     newLeftRearTarget;
        int     newRightFrontTarget;
        int     newRightRearTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  accelError;
        double  steer;
        double  accelSteer;
        double  leftFrontSpeed;
        double  leftRearSpeed;
        double  rightFrontSpeed;
        double  rightRearSpeed;
        double  rampUpSpeed = speedIncr;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Determine new target position, and pass to motor controller
            //  The target positions for the leftFront and rightRear wheels are adjust by the
            //  DRIFT_ADJUST (< 1) as these are the wheels that cause the robot to drift to the right
            moveCounts = (int)(distance * COUNTS_PER_INCH/Math.cos(WHEEL_ANGLE)/2);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(moveCounts * DRIFT_ADJUST);
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(moveCounts * DRIFT_ADJUST);


            // Set Target and Turn On RUN_TO_POSITION
            leftFront.setTargetPosition(newLeftFrontTarget);
            leftRear.setTargetPosition(-newLeftRearTarget);
            rightFront.setTargetPosition(-newRightFrontTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //   The speed for the leftFront and rightRear wheels are adjusted by DRIFT_ADJUST (< 1)
            //   as these are the wheels that cause the robot to drift to the right.
            speed = Range.clip(Math.abs(DRIVE_SPEED /Math.cos(WHEEL_ANGLE)/2), 0.0, 1.0);            // This while loop ensures that the motor speeds are ramped up slowly.  Setting the speeds
            //   suddenly causes the wheels to slip which in turn mess up the distance traveled.
            while (rampUpSpeed < speed) {
                leftFront.setPower(rampUpSpeed * DRIFT_ADJUST);
                leftRear.setPower(rampUpSpeed);
                rightFront.setPower(rampUpSpeed);
                rightRear.setPower(rampUpSpeed * DRIFT_ADJUST);
                rampUpSpeed += SPEED_INCR;
            }


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
                }

                rightFrontSpeed = speed - steer;
                rightRearSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightRearSpeed));
                if (max > 1.0)
                {
                    rightFrontSpeed /= max;
                    rightRearSpeed /= max;
                }

                leftFrontSpeed = rightFrontSpeed;
                leftRearSpeed = rightRearSpeed;

                leftFront.setPower(leftFrontSpeed * DRIFT_ADJUST);
                leftRear.setPower(-leftRearSpeed);
                rightFront.setPower(-rightFrontSpeed);
                rightRear.setPower(rightRearSpeed * DRIFT_ADJUST);

            }

            // Stop all motion;
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData(">", gyro.getAngularOrientation().firstAngle);
            telemetry.update();
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the request edtime has elapsed
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
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    public void baseGrabbers(boolean twist){ // Twisters
        // twisters
        if (twist == true) { // twist the things down to grab base
            rightTwist.setPosition(0.5);
            leftTwist.setPosition(0.5);
        } else{
            rightTwist.setPosition(0);
            leftTwist.setPosition(1);
        }
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
        double leftFrontSpeed;
        double leftRearSpeed;
        double rightFrontSpeed;
        double rightRearSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftFrontSpeed  = 0.0;
            leftRearSpeed   = 0.0;
            rightFrontSpeed = 0.0;
            rightRearSpeed = 0.0;

            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightFrontSpeed  = speed * steer;      // <-- MADE THESE NEGATIVE
            leftFrontSpeed   = -rightFrontSpeed;

            rightRearSpeed  = speed * steer;
            leftRearSpeed   = -rightRearSpeed;
        }

        // Send desired speeds to motors.
        leftFront.setPower(leftFrontSpeed);
        leftRear.setPower(leftRearSpeed);
        rightFront.setPower(rightFrontSpeed);
        rightRear.setPower(rightRearSpeed);

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
        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
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


    public void parkRight(double driveSpeed, double turnSpeed){  //Will strafe left until under bridge when placed on RIGHT SIDE OF BLUE or RIGHT SIDE OF RED
        gyroDrive(driveSpeed, SPEED_INCR, 3, 0);
        gyroStrafe(driveSpeed, SPEED_INCR,25, 0);
    }





}