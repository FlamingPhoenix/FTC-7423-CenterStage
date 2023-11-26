package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class liftAnd4barServotest extends LinearOpMode {
    //NO LIFT YET
    DcMotor liftl,liftr, intake;
    Servo claw, arml, armr;


    float encoderDifference;
    int liftposl, liftposr = 0;
    double min = 0;
    double max = 537;


    double liftPower;
    double liftrPower;
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
        liftr.setDirection(DcMotor.Direction.REVERSE);
        liftl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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



            liftposl = liftl.getCurrentPosition();
            liftposr = liftr.getCurrentPosition();

            if(gamepad1.right_stick_y>0.1){//up
                liftPower = gamepad1.right_stick_y/1;
            }else if(gamepad1.right_stick_y<-0.1) {//down
                liftPower = gamepad1.right_stick_y/3;
            }else{
                liftPower = 0.1;
            }

            encoderDifference = (float) (liftposl-liftposr);
            liftrPower = liftPower + encoderDifference / 500;
            if(liftposl >= max){
                liftPower = Math.min(liftPower, 0);
                liftrPower = Math.min(liftrPower, 0);
            }
            if(liftposl <= min){
                liftPower = Math.max(liftPower, 0);
                liftrPower = Math.max(liftrPower, 0);
            }

            liftl.setPower(liftPower);
            liftr.setPower(liftrPower);

            telemetry.addData("lift",String.format("l: %d; r: %d", liftl.getCurrentPosition(), liftr.getCurrentPosition()));
            telemetry.addData("encoder difference", encoderDifference);
            telemetry.update();
        }
    }
}
