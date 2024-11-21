package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Yellow Blocks (Blue)")
public class YellowBlocksBlue extends RobotOperator {
    @Override
    public void runOpMode() {
        initializeRobot();
        waitForStart();

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
