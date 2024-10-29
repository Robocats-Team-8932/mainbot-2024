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
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Arrays;

@TeleOp(name="Main2024")
public class Main2024 extends OpMode {
    // Declare our motor variables
    DcMotor LF, LR, RF, RR, Hand, Elbow, Shoulder;
    static double TICKS_TO_IMPERIAL_THING = 118.83569;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize motors
        LF     = hardwareMap.get(DcMotor.class, "LF");
        LR     = hardwareMap.get(DcMotor.class, "LR");
        RF     = hardwareMap.get(DcMotor.class, "RF");
        RR     = hardwareMap.get(DcMotor.class, "RR");

        Hand = hardwareMap.get(DcMotor.class, "Intake");
        Elbow  = hardwareMap.get(DcMotor.class, "Elbow");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");

        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
        tetrixConfig.setGearing(52);
        tetrixConfig.setMaxRPM(165);
        tetrixConfig.setTicksPerRev(1440);
        tetrixConfig.setOrientation(Rotation.CW);

        MotorConfigurationType revConfig = new MotorConfigurationType();
        revConfig.setGearing(40);
        revConfig.setMaxRPM(150);
        revConfig.setTicksPerRev(1120);
        revConfig.setOrientation(Rotation.CCW);

        LF.setMotorType(revConfig);
        LR.setMotorType(revConfig);
        RF.setMotorType(revConfig);
        RR.setMotorType(revConfig);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        Elbow.setMotorType(tetrixConfig);
        Shoulder.setMotorType(tetrixConfig);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        telemetry.addData("Status", "Activating");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Put joystick values into variables
        float ly1 = -gamepad1.left_stick_y;
        float rx1 = gamepad1.right_stick_x;
        float lx1 = gamepad1.left_stick_x;
        float ly2 = -gamepad2.left_stick_y;
        float ry2 = -gamepad2.right_stick_y;

        // Drive control
        if (Math.abs(ly1) > .1) {
            // Forward and backward movement
            LF.setPower(ly1);
            LR.setPower(ly1);
            RF.setPower(ly1);
            RR.setPower(ly1);
        } else if (Math.abs(rx1) > .1) {
            // Rotation
            LF.setPower(rx1);
            LR.setPower(rx1);
            RF.setPower(-rx1);
            RR.setPower(-rx1);
        } else if (Math.abs(lx1) > .1) {
            // Sideways movement
            LF.setPower(-lx1);
            LR.setPower(lx1);
            RF.setPower(lx1);
            RR.setPower(-lx1);
        } else {
            // Stop motors if no input
            LF.setPower(0);
            LR.setPower(0);
            RF.setPower(0);
            RR.setPower(0);
        }

        if (Math.abs(ly2) > .1)
            Shoulder.setPower(ly2);
        else
            Shoulder.setPower(0);

        if (Math.abs(ry2) > .1)
            Elbow.setPower(ry2);
        else
            Elbow.setPower(0);

        if (Math.abs(gamepad2.left_trigger) > .2)
            Hand.setPower(-.8);
        else if (Math.abs(gamepad2.right_trigger) > .2)
            Hand.setPower(.8);
        else
            Hand.setPower(0);

        // Update dashboard values
        telemetry.addData("Encoders", Arrays.toString(new int[]{
                LF.getCurrentPosition(),
                LR.getCurrentPosition(),
                RF.getCurrentPosition(),
                RR.getCurrentPosition()
        }));
        telemetry.addData("Controller One", Arrays.toString(new float[]{
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
        }));
        telemetry.addData("Controller Two", Arrays.toString(new float[]{
                gamepad2.left_stick_x,
                gamepad2.left_stick_y,
                gamepad2.left_trigger,
                gamepad2.right_trigger
        }));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Stop motors for safety
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);

        Shoulder.setPower(0);
        Elbow.setPower(0);
        Hand.setPower(0);
    }

}
