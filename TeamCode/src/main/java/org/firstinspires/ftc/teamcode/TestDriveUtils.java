package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveUtilities.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.DriveUtilities.BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.DriveUtilities.FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.DriveUtilities.FRONT_RIGHT;
import static org.firstinspires.ftc.teamcode.DriveUtilities.driveForInches;
import static org.firstinspires.ftc.teamcode.DriveUtilities.initializeDriveTrain;
import static org.firstinspires.ftc.teamcode.DriveUtilities.stopDrive;
import static org.firstinspires.ftc.teamcode.DriveUtilities.telemetryNextDataOnWait;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name="TestUtils")
public class TestDriveUtils extends OpMode {
    DcMotor hand, elbow, shoulder;

    DcMotor[] driveTrain;

    @Override
    public void init() {
        // Initialize motors
        driveTrain = initializeDriveTrain(hardwareMap);

        hand = hardwareMap.get(DcMotor.class, "Intake");
        elbow  = hardwareMap.get(DcMotor.class, "Elbow");
        shoulder = hardwareMap.get(DcMotor.class, "Shoulder");

        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
        tetrixConfig.setGearing(52);
        tetrixConfig.setMaxRPM(165);
        tetrixConfig.setTicksPerRev(1440);
        tetrixConfig.setOrientation(Rotation.CW);

        elbow.setMotorType(tetrixConfig);
        shoulder.setMotorType(tetrixConfig);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            driveForInches(driveTrain, .5f, 10f);
        }


        if (gamepad1.b) {
            telemetryNextDataOnWait = new Object[][] {
                    { "My cool caption", 1 }
            };

            DriveUtilities.driveSidewaysForInches(driveTrain, .25f, 10f);
        }

        if (gamepad1.x) {
            DriveUtilities.rotateForDegrees(driveTrain, .5f, 90f);
        }
    }

    @Override
    public void stop() {
        stopDrive(driveTrain);

        elbow.setPower(0);
        hand.setPower(0);
        shoulder.setPower(0);
    }
}
