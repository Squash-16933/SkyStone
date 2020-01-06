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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SQUISH-Heading", group="Iterative Opmode")
public class HeadingDrive extends OpMode {

    /*
     * Create position and delta variables inside TeleOp class
     * create DcMotors, controllers, and servos
     */

    private BNO055IMU gyro;
    private Orientation angles;
    private Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0

    private boolean isStart = false;

    final double PI = 3.1415;

//    private Servo rightTwist = null;
//    private Servo leftTwist = null;
//
//    private Servo rampServo = null;
//    private double rampPos = 0.4;
//
//    private DcMotor rightIntake = null; // port 0
//    private DcMotor leftIntake = null; // port 1
//
//    private DigitalChannel stoneStop = null;
//    private static final double RAMP_SERVO_INCREMENT = 0.001;

    @Override
    public void init() {

        /*
         * Code to run ONCE when the driver hits INIT
         */

        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        /*
         * Declare drive train motors
         */
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

//        rightTwist = hardwareMap.get(Servo.class, "rightTwist");
//        leftTwist = hardwareMap.get(Servo.class, "leftTwist");
//
//        rampServo = hardwareMap.get(Servo.class, "rampServo");
//        rampServo.setPosition(rampPos);
//
//        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
//        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");

        /*
         * Initialize motors, servos, and controllers with hardwareMap
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // while the power of the wheels is 0, brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        //gyro.calibrate();
        gyro.initialize(parameters);
        // make sure the gyro is calibrated before continuing
        while (!isStart && gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        //Intake setup
//        rightIntake.setDirection(DcMotor.Direction.FORWARD);
//        leftIntake.setDirection(DcMotor.Direction.REVERSE);


//        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set twisters to up position
//        rightTwist.setPosition(0);
//        leftTwist.setPosition(1);

        //Intake stopper
//        stoneStop = hardwareMap.get(DigitalChannel.class, "stoneStop");
//        stoneStop.setMode(DigitalChannel.Mode.INPUT);
    }


    @Override
    public void init_loop() {

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */

        telemetry.addLine("Waiting to start");
        telemetry.addData("Heading: ", gyro.getAngularOrientation().firstAngle);
        telemetry.update();

    }


    @Override
    public void start() {

        /*
         * Code to run ONCE when the driver hits PLAY
         */

        isStart = true;

    }


    @Override
    public void loop() {

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */

        double desiredHeading;
        double velocityMag;
        double velocityAng;
        double control = 1.75;

        if (gamepad1.right_bumper) {
            control = 3;
        } else if (gamepad1.left_bumper) {
            control = 1;
        }

        double driveY = -gamepad1.left_stick_y;
        double driveX =  gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

        velocityMag = Math.sqrt((driveX*driveX)+(driveY*driveY));
        velocityAng = findAngle(driveX, driveY) + getError(0);

        driveX = velocityMag * Math.cos(velocityAng);
        driveY = velocityMag * Math.sin(velocityAng);

        double leftFrontPower = Range.clip((driveX - driveY) + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip((driveX + driveY) - turn, -1.0, 1.0);
        double leftRearPower = Range.clip((driveX + driveY) + turn, -1.0, 1.0);
        double rightRearPower = Range.clip((driveX - driveY) - turn, -1.0, 1.0);

        telemetry.addData("Velocity", " magnitude = %f and angle = %f", velocityMag, velocityAng*180/PI);
        telemetry.addData("Speeds", " leftFront = %f and rightFront = %f", leftFrontPower, rightFrontPower);
        telemetry.update();

        leftFront.setPower(leftFrontPower / control);
        leftRear.setPower(leftRearPower / control);
        rightFront.setPower(rightFrontPower / control);
        rightRear.setPower(rightRearPower / control);

    }

    @Override
    public void stop() {

        /*
         * Code to run ONCE after the driver hits STOP
         */
        // stop all motion
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }

    public double findAngle(double x, double y) {
        double angle = Math.atan(x/y);

        if (y <= 0) {
            if (x < 0) {
                angle -= PI;
            }
            else {
                angle += PI;
            }
        }

        if (x==0 && y==0){
            angle = 0;
        }

        return -angle;
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
        while (robotError > PI)  robotError -= 2*PI;
        while (robotError <= -PI) robotError += 2*PI;
        return robotError;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
}
