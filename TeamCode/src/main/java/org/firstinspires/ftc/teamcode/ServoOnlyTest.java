package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoOnlyTest extends OpMode {
    Claw claw;
    Servo clawServo;
    double servoPos = 0.5;
    @Override
    public void init() {
        clawServo = hardwareMap.servo.get("arml");
        claw = new Claw(clawServo,0.25,0.27,0.3,0.42);//0.25,0.27,0.3,4.2
        claw.setPos(0.5);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            claw.open();
        }
        if(gamepad1.b){
            claw.close();
        }
        clawServo.setPosition(gamepad1.left_stick_y/2+0.5);
        telemetry.addData("Servo Pos",clawServo.getPosition());
    }
}
