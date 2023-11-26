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
    double minLiftHeight = 0;
    double maxLiftHeight = 537;


    double liftPower;
    double liftrPower;
    @Override
    public void runOpMode(){
        liftl = hardwareMap.dcMotor.get("liftl");
        liftr = hardwareMap.dcMotor.get("liftr");
        intake = hardwareMap.dcMotor.get("intake");

        claw = hardwareMap.servo.get("claw");
        claw.setPosition(1);

        liftr.setDirection(DcMotor.Direction.REVERSE);
        liftl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            //Positive power is going up
            //Negative power is going down

            if(gamepad1.right_stick_y>0.1){
                liftPower = gamepad1.right_stick_y/1;
            }else if(gamepad1.right_stick_y<-0.1) {
                liftPower = gamepad1.right_stick_y/3;
            }else{
                liftPower = 0.1;
            }

            //Adjust the right lift motor's (liftr) power based on encoder difference
            //This helps make sure that the physical lift stays level
            encoderDifference = (float) (liftposl-liftposr);
            liftrPower = liftPower + encoderDifference / 500; //500 is arbitrarily chosen

            //Make sure lift motors can't run past the physical limit of the lift
            if(liftposl >= maxLiftHeight){
                liftPower = Math.min(liftPower, 0);
                liftrPower = Math.min(liftrPower, 0);
            }
            if(liftposl <= minLiftHeight){
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
