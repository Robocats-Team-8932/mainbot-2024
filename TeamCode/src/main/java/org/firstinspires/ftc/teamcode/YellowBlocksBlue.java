package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveUtilities.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@Autonomous(name="Yellow Blocks (Blue)")
public class YellowBlocksBlue extends LinearOpMode {
    static DcMotor[] driveTrain;
    static DcMotor Hand, Elbow, Shoulder;

    @Override
    public void runOpMode() {
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

        // FIXME: Keep note of this
        resetEncoders(driveTrain);

        DriveUtilities.telemetry = telemetry;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Do the auton thing!!@

        // Get in position
        telemetry.addData("Start Drive", true);
        telemetry.update();
        driveForInches(driveTrain, .4f, 10);

        telemetry.addData("Start Wait", true);
        telemetry.update();
        waitForMotorPositionReached();

        telemetry.addData("Start Rotate", true);
        telemetry.update();
        rotateForDegrees(driveTrain, .4f, 20);
        waitForMotorPositionReached();

//        // Roll out arm
//        powerForSeconds(Shoulder, .3f, .5f);
//        powerForSeconds(Elbow, .3f, .4f);
//
//        // Run outtake
//        powerForSeconds(Hand, 1, .2f);
//
//        // Pull arm back in
//        powerForSeconds(Shoulder, -.3f, .5f);
//        powerForSeconds(Elbow, -.3f, .4f);

        // Stop!!!0
        terminateOpModeNow();
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
            telemetry.addData("Waiting", true);
        }
        resetMotorMode(driveTrain);
    }
}
