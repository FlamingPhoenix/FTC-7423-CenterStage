package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TeleOp2 extends OpMode {
    DcMotor fl, fr, bl, br, intake, liftl, liftr;
    Servo clawServo,arml,armr;
    IMU imu;
    Claw claw;
    Lift lift;
    ServoArm servoArm;
    armAssemblyTeleOp armAssembly;
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

        armAssembly = new armAssemblyTeleOp(claw, servoArm, lift, gamepad1, gamepad2);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }
    @Override
    public void loop(){
        double y = -gamepad1.left_stick_y;// Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        double botHeading = -imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        intake.setPower(gamepad2.right_trigger);

        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double flp = -(rotY + rotX + rx) / denominator;
        double blp = -(rotY - rotX + rx) / denominator;
        double frp = -(rotY - rotX - rx) / denominator;
        double brp = -(rotY + rotX - rx) / denominator;

        flp = flp * (1 - gamepad1.right_trigger);
        blp = blp * (1 - gamepad1.right_trigger);
        frp = frp * (1 - gamepad1.right_trigger);
        brp = brp * (1 - gamepad1.right_trigger);
        flp = flp * (1 + 2*gamepad1.left_trigger);
        blp = blp * (1 + 2*gamepad1.left_trigger);
        frp = frp * (1 + 2*gamepad1.left_trigger);
        brp = brp * (1 + 2*gamepad1.left_trigger);

        fl.setPower(0.49*flp);
        bl.setPower(0.49*blp);
        fr.setPower(0.49*frp);
        br.setPower(0.49*brp);

        armAssembly.execute();

        telemetry.update();
    }
}
