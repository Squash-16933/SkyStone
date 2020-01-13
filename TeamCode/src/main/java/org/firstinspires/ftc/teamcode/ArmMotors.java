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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SQUISH ARM", group="Iterative Opmode")
public class ArmMotors extends OpMode {
    private DcMotor fastMotor = null;
    private DcMotor slowMotor = null;
    final static double FAST_MOTOR_TPR = 537.6;
    final static double SLOW_MOTOR_TPR = 383.6;
    final static double SLOW_MOTOR_RPM = 312.0;
    final static double FAST_MOTOR_RPM = 435.0;
    final static double SLOW_MOTOR_SPEED = 0.25;
    final static double FAST_MOTOR_SPEED = SLOW_MOTOR_SPEED * SLOW_MOTOR_RPM / FAST_MOTOR_RPM;
    final static double ROTATION_AMOUNT = 0.5;
    /*
     * Create position and delta variables inside TeleOp class
     * create DcMotors, controllers, and servos
     */

    @Override
    public void init() {
        /*
         * Code to run ONCE when the driver hits INIT
         */

        fastMotor = hardwareMap.get(DcMotor.class, "fastMotor");
        slowMotor = hardwareMap.get(DcMotor.class, "slowMotor");

        fastMotor.setDirection(DcMotor.Direction.REVERSE);
        slowMotor.setDirection(DcMotor.Direction.FORWARD);

        fastMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fastMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        slowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fastMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    @Override
    public void init_loop() {

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */

        /*
         * Set initial servo positions
         */

    }


    @Override
    public void start() {

        /*
         * Code to run ONCE when the driver hits PLAY
         */

    }


    @Override
    public void loop() {
        int slowCount = 0 , fastCount = 0;

//        if (gamepad1.dpad_up) {
//            slowCount = (int)(ROTATION_AMOUNT * SLOW_MOTOR_TPR);
//            fastCount = (int)(ROTATION_AMOUNT * FAST_MOTOR_TPR);
//            slowMotor.setTargetPosition(slowCount);
//            fastMotor.setTargetPosition(fastCount);
//            fastMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slowMotor.setPower(SLOW_MOTOR_SPEED);
//            fastMotor.setPower(FAST_MOTOR_SPEED);
//        }
//        else if (gamepad1.dpad_down) {
//            slowCount = 0;
//            fastCount = 0;
//            slowMotor.setTargetPosition(slowCount);
//            fastMotor.setTargetPosition(fastCount);
//            fastMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slowMotor.setPower(SLOW_MOTOR_SPEED);
//            fastMotor.setPower(FAST_MOTOR_SPEED);
//        }
//
//        while (slowMotor.isBusy() || fastMotor.isBusy()){
//            telemetry.addData("Motors: ", "Slow speed = %4.2f, fast speed = $4.2f",
//                               SLOW_MOTOR_SPEED, FAST_MOTOR_SPEED);
//            telemetry.addData("Motors: ", "Slow target = %d, fast target = $d",
//                    slowCount, fastCount);
//            telemetry.update();
//        }

        if (gamepad1.a) {
            slowMotor.setPower(SLOW_MOTOR_SPEED);
            fastMotor.setPower(FAST_MOTOR_SPEED);
            telemetry.addLine("Pushing a");
            telemetry.addData("slowMotor power", "Desired speed = %4.2f, actual speed = %4.2f",
                               SLOW_MOTOR_SPEED, slowMotor.getPower());
            telemetry.addData("fastMotor power", "Desired speed = %4.2f, actual speed = %4.2f",
                               FAST_MOTOR_SPEED, fastMotor.getPower());
        } else {
            slowMotor.setPower(0);
            fastMotor.setPower(0);
            telemetry.addLine("not pushing anything");
            telemetry.addData("slowMotor power", "Desired speed = %4.2f, actual speed = %4.2f",
                    SLOW_MOTOR_SPEED, slowMotor.getPower());
            telemetry.addData("fastMotor power", "Desired speed = %4.2f, actual speed = %4.2f",
                    FAST_MOTOR_SPEED, fastMotor.getPower());
        }

        telemetry.update();

//        fastMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slowMotor.setPower(0);
//        fastMotor.setPower(0);

    }


    @Override
    public void stop() {


    }

}
