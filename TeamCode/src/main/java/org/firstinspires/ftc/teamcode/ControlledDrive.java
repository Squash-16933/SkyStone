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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SQUISH", group="Iterative Opmode")
public class ControlledDrive extends OpMode {

    /*
     * Create position and delta variables inside TeleOp class
     * create DcMotors, controllers, and servos
     */

    private BNO055IMU gyro;
    private Orientation angles;
    private Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private boolean isStart = false;

    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0

    private Servo rightTwist = null;
    private Servo leftTwist = null;

    private Servo rampServo = null;
    private double rampPos = 1;

    private DcMotor rightIntake = null; // port 0
    private DcMotor leftIntake = null; // port 1

    private double velocityLeftX;
    private double velocityLeftY;

    private double velocityRightX;
    private double velocityRightY;

    private double robotVelocityMag;
    private double robotAngleError;

    private double phi;

    private DigitalChannel stoneStop = null;
    private static final double RAMP_SERVO_INCREMENT = 0.001;

    @Override
    public void init() {

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        /*
         * Code to run ONCE when the driver hits INIT
         */
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        rightTwist = hardwareMap.get(Servo.class, "rightTwist");
        leftTwist = hardwareMap.get(Servo.class, "leftTwist");

        rampServo = hardwareMap.get(Servo.class, "rampServo");
        rampServo.setPosition(rampPos);

        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
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

        //Intake setup
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);


        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set twisters to up position
        rightTwist.setPosition(0);
        leftTwist.setPosition(1);

        //Intake stopper
        stoneStop = hardwareMap.get(DigitalChannel.class, "stoneStop");
        stoneStop.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.initialize(parameters);


        while (!isStart && gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }
    }


    @Override
    public void init_loop() {

       telemetry.addLine("Waiting to start");
       telemetry.addData("Heading: ", gyro.getAngularOrientation().firstAngle);
       telemetry.update();

    }


    @Override
    public void start() {

        /*
         * Code to run ONCE when the driver hits PLAY
         */


    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        double control = 2;

        if (gamepad1.right_bumper) {
            control = 4;
        } else if (gamepad1.left_bumper) {
            control = 1;
        }

        double leftStickY = -gamepad1.left_stick_y;  // Get the Y position of left gamepad stick
        double leftStickX =  gamepad1.left_stick_x;  // Get the X position of left gamepad stick
        double turn   =  gamepad1.right_stick_x; // Get the X position of right gamepad stick

        // Set the velocity of the robot to how far the left gamepad stick is being pressed
        robotVelocityMag = Math.sqrt((leftStickX * leftStickX) + (leftStickY * leftStickY));

        // Set the angle of the robot to the angle of the left gamepad stick
        double robotVelocityAng = getAngle(leftStickX, leftStickY) + Math.toRadians(getError(0));

        double motorX = Math.cos(robotVelocityAng) * robotVelocityMag;
        double motorY = Math.sin(robotVelocityAng) * robotVelocityMag;


        double leftFrontPower  = Range.clip((motorX - motorY) + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip((motorX + motorY) - turn, -1.0, 1.0);
        double leftRearPower   = Range.clip((motorX + motorY) + turn, -1.0, 1.0);
        double rightRearPower  = Range.clip((motorX - motorY) - turn, -1.0, 1.0);

        leftFront.setPower(leftFrontPower / control);
        leftRear.setPower(leftRearPower / control);
        rightFront.setPower(rightFrontPower / control);
        rightRear.setPower(rightRearPower / control);

        /*
         * Set controls on gamepads and update/set position of servos with delta variables
         *
         */

        // twisters
        if (gamepad1.dpad_down) {
            rightTwist.setPosition(0.5);
            leftTwist.setPosition(0.5);
        } else if (gamepad1.dpad_up) {
            rightTwist.setPosition(0);
            leftTwist.setPosition(1);
        }

        //ramp servo
        if (gamepad2.y) {
            //rampPos += RAMP_SERVO_INCREMENT;
            rampPos = 1;
        } else if (gamepad2.a) {
            //rampPos -= RAMP_SERVO_INCREMENT;
            rampPos = 0.15;
        } else if (gamepad2.x) {
            rampPos = 0.8;
        }
        else if(gamepad2.b){
            rampPos = 0.45;
        }

//        if (rampPos > 1) {
//            rampPos = 1;
//        } else if (rampPos < 0) {
//            rampPos = 0;
//        }
        telemetry.addData("Ramp Position", rampPos);
        telemetry.update();
        rampServo.setPosition(rampPos);

        //Intake/Outtake
        if(gamepad2.left_trigger>0) {
            if(stoneStop.getState() == false){
                intake(true, false, gamepad2.right_trigger, true);
            }
            else{
                intake(true, true, gamepad2.left_trigger, false);
            }
        }
        else if(gamepad2.right_trigger>0) {
                intake(true, false, gamepad2.right_trigger, false);
        }
        else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        telemetry.update();

    }

    public void intake(boolean on, boolean in, float power, boolean stop) {
        if (on == true) {
            if (in) {
                rightIntake.setPower(power);
                leftIntake.setPower(power);
            }
            else if(stop == true){
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }
            else {
                rightIntake.setPower(-power);
                leftIntake.setPower(-power);
            }
        }
    }

    /**
     * Takes the x and y position of the left gamepad stick and outputs its angle.
     * @param x The x position
     * @param y The y position
     * @return The angle
     */
    public double getAngle(double x, double y){
        double angle = Math.atan(x/y);

        if (y <= 0) {
            if (x < 0) {
                angle -= Math.PI;
            }
            else {
                angle += Math.PI;
            }
        }

        if (x==0 && y==0){
            angle = 0;
        }

        return -angle;
    }


    /**
     * Code to run ONCE after the driver hits STOP, to stop all motion
     */
    @Override
    public void stop() {
        // stop all motion
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
        // Return claw to normal position
    }

    public double getError(double targetAngle) {


        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public final void sleep(long milliseconds){
        try{
            Thread.sleep(milliseconds);
        }   catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }

    public final void idle(){
        Thread.yield();
    }

}
