package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class DriveUtilities {
    static double INCHES_2_TICKS = 118.83569;

    static byte FRONT_LEFT = 0;
    static byte FRONT_RIGHT = 1;
    static byte BACK_LEFT = 2;
    static byte BACK_RIGHT = 3;

    public static Telemetry telemetry;

    private static void setDestinationTicks(DcMotor[] motors, int ticks) {
        // Set destination position and mode
        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

//    public static void powerForSeconds(DcMotor motor, float power, float seconds) {
//        motor.setPower(power);
//        waitSeconds(seconds);
//        motor.setPower(0);
//    }

    public static void drive(DcMotor[] motors, float speed) {
        for (DcMotor motor : motors)
            motor.setPower(speed);
    }

    public static void stopDrive(DcMotor[] motors) {
        drive(motors, 0);
    }

    /*
        Drive forward for the specified amount of inches.
    */
    public static void driveForInches(DcMotor[] motors, float speed, float inches) {


        telemetry.addData("A", 1);
        telemetry.update();

        int ticks = (int)Math.floor(INCHES_2_TICKS * inches);
        setDestinationTicks(motors, ticks);

        telemetry.addData("B", 1);
        telemetry.update();

        drive(motors, speed);

        telemetry.addData("C", 1);
        telemetry.update();
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
        setDestinationTicks(motors, ticks);
        driveSideways(motors, speed);
    }

    public static void rotate(DcMotor[] motors, float speed) {
        motors[FRONT_LEFT].setPower(speed);
        motors[BACK_LEFT].setPower(speed);
        motors[FRONT_RIGHT].setPower(-speed);
        motors[BACK_RIGHT].setPower(-speed);
    }

    public static void rotateForDegrees(DcMotor[] motors, float speed, float degrees) {
        int ticks = (int) Math.floor(INCHES_2_TICKS * (16 * Math.PI * (degrees/360)));
        setDestinationTicks(motors, ticks);
        rotate(motors, speed);
    }

    public static void resetMotorMode(DcMotor[] motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * I think this has to be run in initialization, some dark fuckery must happen there
     * @param motors The motors to reset
     */
    public static void resetEncoders(DcMotor [] motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static DcMotor[] initializeDriveTrain(HardwareMap map) {
        DcMotor[] motors = new DcMotor[4];

        motors[FRONT_LEFT] = map.get(DcMotor.class, "LF");
        motors[BACK_LEFT] = map.get(DcMotor.class, "LR");
        motors[FRONT_RIGHT] = map.get(DcMotor.class, "RF");
        motors[BACK_RIGHT] = map.get(DcMotor.class, "RR");

//        MotorConfigurationType tetrixConfig = new MotorConfigurationType();
//        tetrixConfig.setGearing(52);
//        tetrixConfig.setMaxRPM(165);
//        tetrixConfig.setTicksPerRev(1440);
//        tetrixConfig.setOrientation(Rotation.CW);

        MotorConfigurationType revConfig = new MotorConfigurationType();
        revConfig.setGearing(40);
        revConfig.setMaxRPM(150);
        revConfig.setTicksPerRev(1120);
        revConfig.setOrientation(Rotation.CCW);

        for (DcMotor motor : motors)
            motor.setMotorType(revConfig);

        motors[FRONT_RIGHT].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[BACK_RIGHT].setDirection(DcMotorSimple.Direction.REVERSE);

        resetMotorMode(motors);
        return motors;
    }

}
