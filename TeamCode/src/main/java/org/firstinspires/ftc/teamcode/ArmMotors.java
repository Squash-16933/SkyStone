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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled

@TeleOp(name="SQUISH-ARM", group="Iterative Opmode")
public class ArmMotors extends OpMode {
    private DcMotor fastMotor = null;
    private DcMotor slowMotor = null;
    final static double fastMotorTPR = 537.6;
    final static double slowMotorTPR = 383.6;
    final static double fastMotorRPM = 312.0;
    final static double slowMotorRPM = 435.0;
    final static double fastMotorSpeed = slowMotorRPM / fastMotorRPM;
    /*
     * Create position and delta variables inside TeleOp class
     * create DcMotors, controllers, and servos
     */

    @Override
    public void init() {
        fastMotor = hardwareMap.get(DcMotor.class, "fastMotor");
        slowMotor = hardwareMap.get(DcMotor.class, "slowMotor");
        /*
         * Code to run ONCE when the driver hits INIT
         */

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


    }


    @Override
    public void stop() {


    }

}
