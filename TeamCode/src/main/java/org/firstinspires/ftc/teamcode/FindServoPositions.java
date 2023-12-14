package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Lift;

@TeleOp
public class FindServoPositions extends OpMode {
    Servo claw,arml,armr;
    DcMotor liftl,liftr;
    Servo[] servos;
    float servoPositions[] = {0.1f,0.5f};
    boolean hasUpdatedPosition = false;
    int whichOne = 0;
    Lift lift;
    @Override
    public void init(){
        arml = hardwareMap.get(Servo.class,"arml");
        armr = hardwareMap.get(Servo.class,"armr");
        claw = hardwareMap.get(Servo.class,"claw");
        liftl = hardwareMap.dcMotor.get("liftl");
        liftr = hardwareMap.dcMotor.get("liftr");
        liftr.setDirection(DcMotor.Direction.REVERSE);
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift = new Lift(liftl,liftr);
        servos = new Servo[]{claw, arml, armr};
        for(Servo servo:servos){
            servo.setPosition(0.5);
        }
    }
    @Override
    public void loop(){
        if(gamepad1.a){
            whichOne = 1;
        }
        if(gamepad1.x){
            whichOne = 0;
        }
        if(gamepad1.dpad_up){
            if(!hasUpdatedPosition){
                servoPositions[whichOne]+=0.01;
                hasUpdatedPosition = true;
            }
        }else if(gamepad1.dpad_down){
            if(!hasUpdatedPosition){
                servoPositions[whichOne]-=0.01;
                hasUpdatedPosition = true;
            }
        }else if(gamepad1.left_stick_y > 0.05){
            if(!hasUpdatedPosition){
                servoPositions[whichOne]+=0.1;
                hasUpdatedPosition = true;
            }
        }else if(gamepad1.left_stick_y < -0.05){
            if(!hasUpdatedPosition){
                servoPositions[whichOne]-=0.1;
                hasUpdatedPosition = true;
            }
        }else{
            hasUpdatedPosition = false;
        }
        lift.setLiftDualMotorPos(25);

        telemetry.addData("clawPos",servoPositions[0]);
        telemetry.addData("leftPos",servoPositions[1]);
        telemetry.addData("rightPos",1-servoPositions[1]);
        telemetry.addData("liftPos",liftl.getCurrentPosition());
        arml.setPosition(servoPositions[1]);
        armr.setPosition(1-servoPositions[1]);
        claw.setPosition(servoPositions[0]);
    }
}
