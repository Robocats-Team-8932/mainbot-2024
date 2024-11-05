package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveUtilities.initializeDriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import static org.firstinspires.ftc.teamcode.DriveUtilities.*;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@Autonomous(name="Number One!")
public class NumberOneAuton extends OpMode {
    DcMotor[] driveTrain;
    DcMotor Hand, Elbow, Shoulder;

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

    @Override
    public void loop() {
        // Do the auton thing!!@

        // drive(driveTrain, speed)
        // driveForInches(driveTrain, speed, inches)
        // rotate(driveTrain, speed)
        // rotateForInches(driveTrain, speed, inches)
        // driveSideways(driveTrain, speed)
        // stopDrive(driveTrain)

        // Elbow.setPower(something), shoulder

        // Get in position
        driveForInches(driveTrain, .4f, 10);
        rotateForDegrees(driveTrain, .4f, 20);

        // Roll out arm
        powerForSeconds(Shoulder, .3f, .5f);
        powerForSeconds(Elbow, .3f, .4f);
        // Run outtake
        powerForSeconds(Hand, 1, .2f);

        // Pull arm back in
        powerForSeconds(Shoulder, -.3f, .5f);
        powerForSeconds(Elbow, -.3f, .4f);


        // Stop!!!0
        terminateOpModeNow();
    }
}
