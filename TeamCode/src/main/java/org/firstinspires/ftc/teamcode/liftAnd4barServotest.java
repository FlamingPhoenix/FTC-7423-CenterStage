package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class liftAnd4barServotest extends LinearOpMode {
    //NO LIFT YET
    DcMotor liftl,liftr, intake;
    Servo claw;
    @Override
    public void runOpMode(){
        liftl = hardwareMap.dcMotor.get("liftl");
        liftr = hardwareMap.dcMotor.get("liftr");
        intake = hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(1);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(1);
            }
            if (gamepad1.b) {
                claw.setPosition(0.9);
            }
            if(gamepad1.right_trigger > 0.1){
                intake.setPower(gamepad1.right_trigger/2);
            }
        }
    }
}
