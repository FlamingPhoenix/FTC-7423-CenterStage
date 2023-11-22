package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class liftAnd4barServotest extends LinearOpMode {
    //NO LIFT YET
    DcMotor liftl,liftr, intake;
    Servo claw, arml, armr;
    @Override
    public void runOpMode(){
        liftl = hardwareMap.dcMotor.get("liftl");
        liftr = hardwareMap.dcMotor.get("liftr");
        intake = hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        //arml = hardwareMap.servo.get("arml");
        //armr = hardwareMap.servo.get("armr");
        //arml.setPosition(0.6);
        //armr.setPosition(0.4);
        claw.setPosition(1);
        liftl.setDirection(DcMotor.Direction.REVERSE);
        liftl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            }else{
                intake.setPower(0);
            }
            if(gamepad1.right_stick_y>0.1){
                liftl.setPower(gamepad1.right_stick_y/3);
                liftr.setPower(gamepad1.right_stick_y/3);

            }else if( gamepad1.right_stick_y<-0.1) {
                liftl.setPower(gamepad1.right_stick_y/1);
                liftr.setPower(gamepad1.right_stick_y/1);
            }else{
                    liftl.setPower(-0.1);
                    liftr.setPower(-0.1);

            }
            telemetry.addData("liftr",liftr.getCurrentPosition());
            telemetry.update();
        }
    }
}
