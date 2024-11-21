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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

@TeleOp(name="Main 2024")
public class Main2024 extends RobotOperator {
    @Override
    public void runOpMode() {
        initializeRobot();
        waitForStart();

        while (opModeIsActive()) {
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
                driveSideways(driveTrain, -lx1);
            else
                // Stop motors if no input
                stopDrive(driveTrain);

            if (Math.abs(ly2) > .1)
                Shoulder.setPower(ly2);
            else
                Shoulder.setPower(0);

            // FIXME: Hardcoded speed limiter
            if (Math.abs(ry2) > .1)
                Elbow.setPower(.5 * ry2);
            else
                Elbow.setPower(0);

            if (Math.abs(gamepad2.left_trigger) > .2)
                Hand.setPower(-1);
            else if (Math.abs(gamepad2.right_trigger) > .2)
                Hand.setPower(1);
            else
                Hand.setPower(0);

            if (gamepad2.left_bumper)
                Climber.setPower(-1);
            else if (gamepad2.right_bumper)
                Climber.setPower(1);
            else
                Climber.setPower(0);

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

        // Stop motors for safety
        stopDrive(driveTrain);

        Shoulder.setPower(0);
        Elbow.setPower(0);
        Hand.setPower(0);
    }
}
