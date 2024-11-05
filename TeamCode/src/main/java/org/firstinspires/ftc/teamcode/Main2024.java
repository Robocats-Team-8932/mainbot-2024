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

import static org.firstinspires.ftc.teamcode.DriveUtilities.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Arrays;

@TeleOp(name="Main2024")
public class Main2024 extends OpMode {
    // Declare our motor variables
    DcMotor[] driveTrain;
    DcMotor Hand, Elbow, Shoulder;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize motors
        driveTrain = initializeDriveTrain(hardwareMap);

        Hand = hardwareMap.get(DcMotor.class, "Intake");
        Elbow  = hardwareMap.get(DcMotor.class, "Elbow");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");

        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
        tetrixConfig.setGearing(52);
        tetrixConfig.setMaxRPM(165);
        tetrixConfig.setTicksPerRev(1440);
        tetrixConfig.setOrientation(Rotation.CW);

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
        if (Math.abs(ly1) > .1)
            // Forward and backward movement
            drive(driveTrain, ly1);

        else if (Math.abs(rx1) > .1)
            // Rotation
            rotate(driveTrain, rx1);

        else if (Math.abs(lx1) > .1)
            // Sideways movement
            driveSideways(driveTrain, lx1);

        else
            // Stop motors if no input
            stopDrive(driveTrain);

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
                driveTrain[FRONT_LEFT].getCurrentPosition(),
                driveTrain[BACK_LEFT].getCurrentPosition(),
                driveTrain[FRONT_RIGHT].getCurrentPosition(),
                driveTrain[BACK_RIGHT].getCurrentPosition()
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
        stopDrive(driveTrain);

        Shoulder.setPower(0);
        Elbow.setPower(0);
        Hand.setPower(0);
    }

}
