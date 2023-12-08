package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOp2 extends OpMode {
    DcMotor fl, fr, bl, br, intake, liftl, liftr;
    Servo clawServo,arml,armr;
    IMU imu;
    Claw claw;
    Lift lift;
    ServoArm servoArm;
    @Override
    public void init(){
        // DC MOTORS // DC MOTORS // DC MOTORS // DC MOTORS // DC MOTORS // DC MOTORS // DC MOTORS // DC MOTORS //
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");

        intake = hardwareMap.dcMotor.get("intake");

        liftr = hardwareMap.dcMotor.get("liftr");
        liftl = hardwareMap.dcMotor.get("liftl");
        // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS // SERVOS //
        clawServo = hardwareMap.servo.get("claw");
        armr = hardwareMap.servo.get("armr");
        arml = hardwareMap.servo.get("arml");

        claw = new Claw(clawServo,0.9,0.8,1,0.6); //NOT FINAL - DO NOT RUN!!!!!!!!!!!!
        servoArm = new ServoArm(arml,armr,0.93,0.3); //NOT FINAL - DO NOT RUN!!!!!!!!!!
        lift = new Lift(liftr,liftl,gamepad2);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }
    @Override
    public void loop(){

    }
}
