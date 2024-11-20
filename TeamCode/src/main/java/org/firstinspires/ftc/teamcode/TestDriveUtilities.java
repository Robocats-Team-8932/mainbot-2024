package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveUtilities.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name="Test Drive Utilities")
public class TestDriveUtilities extends LinearOpMode {
    DcMotor Hand, Elbow, Shoulder;

    DcMotor[] driveTrain;

    @Override
    public void runOpMode() {
        // Initialize motors
        driveTrain = initializeDriveTrain(hardwareMap);

        Hand = hardwareMap.get(DcMotor.class, "Intake");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");

        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
        tetrixConfig.setGearing(52);
        tetrixConfig.setMaxRPM(165);
        tetrixConfig.setTicksPerRev(1440);
        tetrixConfig.setOrientation(Rotation.CW);

        Elbow.setMotorType(tetrixConfig);
        Shoulder.setMotorType(tetrixConfig);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                driveForInches(driveTrain, .5f, 10f);
                waitForMotorPositionReached();
            }

            if (gamepad1.b) {
                driveSidewaysForInches(driveTrain, .25f, 10f);
                waitForMotorPositionReached();
            }
            if (gamepad1.x) {
                rotateForDegrees(driveTrain, .5f, 90f);
                waitForMotorPositionReached();
            }
        }

        stopDrive(driveTrain);

        Elbow.setPower(0);
        Hand.setPower(0);
        Shoulder.setPower(0);
    }

    public void waitForMotorPositionReached() {
        while (
                driveTrain[0].isBusy()
                        || driveTrain[1].isBusy()
                        || driveTrain[2].isBusy()
                        || driveTrain[3].isBusy()
        ) {
            if (!opModeIsActive())
                terminateOpModeNow();

        }
        resetMotorMode(driveTrain);
    }
}
