package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import java.util.Arrays;
import java.util.concurrent.Callable;

public class RobotOperator extends LinearOpMode {
    DcMotor[] driveTrain;
    DcMotor Hand, Elbow, Shoulder, Climber;

    void initializeRobot() {
        driveTrain = initializeDriveTrain(hardwareMap, this::opModeIsActive, this::terminateOpModeNow, telemetry);
        resetDriveTrain(driveTrain);

        Hand = hardwareMap.get(DcMotor.class, "Intake");
        Elbow  = hardwareMap.get(DcMotor.class, "Elbow");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        Climber = hardwareMap.get(DcMotor.class, "Climber");

        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
        tetrixConfig.setGearing(52);
        tetrixConfig.setMaxRPM(165);
        tetrixConfig.setTicksPerRev(1440);
        tetrixConfig.setOrientation(Rotation.CW);

        Elbow.setMotorType(tetrixConfig);
        Elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder.setMotorType(tetrixConfig);
        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    static double INCHES_2_TICKS = 118.83569;

    static byte
            FRONT_LEFT = 0,
            FRONT_RIGHT = 1,
            BACK_LEFT = 2,
            BACK_RIGHT = 3;

    static Callable<Boolean> OPMODE_ACTIVE_CHECK_HANDLE;
    static Runnable OPMODE_TERMINATE_HANDLE;
    static Telemetry OPMODE_TELEMETRY;


    private static void awaitMotorsNotBusy(DcMotor[] motors) {
        // "code"
        while (
                motors[0].isBusy()
                        && motors[1].isBusy()
                        && motors[2].isBusy()
                        && motors[3].isBusy()
        ) {
            try {
                if (!OPMODE_ACTIVE_CHECK_HANDLE.call())
                    OPMODE_TERMINATE_HANDLE.run();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            OPMODE_TELEMETRY.addData("Encoders", Arrays.toString(new int[]{
                    motors[FRONT_LEFT].getCurrentPosition(),
                    motors[BACK_LEFT].getCurrentPosition(),
                    motors[FRONT_RIGHT].getCurrentPosition(),
                    motors[BACK_RIGHT].getCurrentPosition()
            }));
            OPMODE_TELEMETRY.update();
        }

        resetDriveTrain(motors);
    }

    public static void waitSeconds(float seconds) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (time.time() < seconds) {
            try {
                if (!OPMODE_ACTIVE_CHECK_HANDLE.call())
                    // FIXME: no working??? what? no broke tho
                    OPMODE_TERMINATE_HANDLE.run();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            OPMODE_TELEMETRY.addData("Remaining Time", seconds - time.time());
            OPMODE_TELEMETRY.update();
        }
    }

    public static void powerForSeconds(DcMotor motor, float power, float seconds) {
        motor.setPower(power);
        waitSeconds(seconds);
        motor.setPower(0);
    }

    public static void drive(DcMotor[] motors, float speed) {
        for (DcMotor motor : motors)
            motor.setPower(speed);
    }

    public static void stopDrive(DcMotor[] motors) {
        drive(motors, 0);
    }

    public static void driveForInches(DcMotor[] motors, float speed, float inches) {
        int ticks = (int)Math.floor(INCHES_2_TICKS * inches);

        if ((inches < 0 && speed > 0) || (inches > 0 && speed < 0))
            speed *= -1;

        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        drive(motors, speed);
        awaitMotorsNotBusy(motors);
    }

    /**
     * A positive value will send the robot right,
     * a negative one will send it left
     */
    public static void driveSideways(DcMotor[] motors, float speed) {
        motors[FRONT_LEFT].setPower(-speed);
        motors[BACK_LEFT].setPower(speed);
        motors[FRONT_RIGHT].setPower(speed);
        motors[BACK_RIGHT].setPower(-speed);
    }

    public static void driveSidewaysForInches(DcMotor[] motors, float speed, float inches) {
        int ticks = (int)Math.floor(INCHES_2_TICKS * inches * 4/3);

        if ((inches < 0 && speed > 0) || (inches > 0 && speed < 0))
            speed *= -1;

        motors[FRONT_LEFT].setTargetPosition(-ticks);
        motors[BACK_LEFT].setTargetPosition(ticks);
        motors[FRONT_RIGHT].setTargetPosition(ticks);
        motors[BACK_RIGHT].setTargetPosition(-ticks);

        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveSideways(motors, speed);
        awaitMotorsNotBusy(motors);
    }

    public static void rotate(DcMotor[] motors, float speed) {
        motors[FRONT_LEFT].setPower(speed);
        motors[BACK_LEFT].setPower(speed);
        motors[FRONT_RIGHT].setPower(-speed);
        motors[BACK_RIGHT].setPower(-speed);
    }

    public static void rotateForDegrees(DcMotor[] motors, float speed, float degrees) {
        int ticks = (int) Math.floor(INCHES_2_TICKS * (16 * Math.PI * (degrees/360)));

        if ((degrees < 0 && speed > 0) || (degrees > 0 && speed < 0))
            speed *= -1;

        motors[FRONT_LEFT].setTargetPosition(ticks);
        motors[BACK_LEFT].setTargetPosition(ticks);
        motors[FRONT_RIGHT].setTargetPosition(-ticks);
        motors[BACK_RIGHT].setTargetPosition(-ticks);

        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotate(motors, speed);
        awaitMotorsNotBusy(motors);
    }

    public static void resetDriveTrain(DcMotor[] motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public static DcMotor[] initializeDriveTrain(
            HardwareMap map,
            Callable<Boolean> opModeActiveCheckHandle,
            Runnable opModeTerminateHandle,
            Telemetry telemetryInstance
    ) {
        OPMODE_ACTIVE_CHECK_HANDLE = opModeActiveCheckHandle;
        OPMODE_TERMINATE_HANDLE = opModeTerminateHandle;
        OPMODE_TELEMETRY = telemetryInstance;

        DcMotor[] motors = new DcMotor[4];
        motors[FRONT_LEFT] = map.get(DcMotor.class, "LF");
        motors[BACK_LEFT] = map.get(DcMotor.class, "LR");
        motors[FRONT_RIGHT] = map.get(DcMotor.class, "RF");
        motors[BACK_RIGHT] = map.get(DcMotor.class, "RR");

        motors[FRONT_RIGHT].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[BACK_RIGHT].setDirection(DcMotorSimple.Direction.REVERSE);


        resetDriveTrain(motors);
        return motors;
    }

    public void runOpMode() {
        // :D
    }
}
