package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FindServoPositions extends OpMode {
    Servo claw,arml,armr;
    Servo[] servos;
    float servoPositions[] = {0.1f,0.5f};
    boolean hasUpdatedPosition = false;
    int whichOne = 0;
    @Override
    public void init(){
        arml = hardwareMap.servo.get("arml");
        armr = hardwareMap.servo.get("armr");
        claw = hardwareMap.servo.get("claw");
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
        telemetry.addData("clawPos",servoPositions[0]);
        telemetry.addData("leftPos",servoPositions[1]);
        telemetry.addData("rightPos",servoPositions[2]);
        arml.setPosition(servoPositions[1]);
        armr.setPosition(1-servoPositions[1]);
        claw.setPosition(servoPositions[0]);
    }
}
