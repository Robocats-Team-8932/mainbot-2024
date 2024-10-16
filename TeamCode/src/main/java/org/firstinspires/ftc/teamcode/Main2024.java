package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Main2024",group="Linear OpMode")
public class Main2024 extends LinearOpMode {
    DcMotor LF,LR,RF,RR;

    public void runOpMode(){
        // Intitialize motors
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");

        telemetry.addData("Init:", "Succes");
        telemetry.update();

        waitForStart();//Wait for the start of the op mode
        while (opModeIsActive()) { //Loop while the op mode is active
            float ly =-gamepad1.left_stick_y; //Invert for forawd motion
            float rx =gamepad1.right_stick_x;
            float lx =gamepad1.left_stick_x;

            //Drive controll
            if (Math.abs(ly)> 0.1){
                //Foword and backward movemant
                LF.setPower(ly);
                LR.setPower(ly);
                RF.setPower(ly);
                RR.setPower(ly);
            }else if (Math.abs(rx)>0.1){
                //Rotate
                LF.setPower(rx);
                LR.setPower(-rx);
                RF.setPower(rx);
                RR.setPower(-rx);
            }else if (Math.abs(lx)>0.1){
                // Sideways movement
                LF.setPower(-lx);
                LR.setPower(lx);
                RF.setPower(lx);
                RR.setPower(-lx);
            }else{
                // Stop motors if no input
                LF.setPower(0);
                LR.setPower(0);
                RF.setPower(0);
                RR.setPower(0);
            }
            telemetry.addData("Robot:","Activating!");
            telemetry.update();
        }
    }
}