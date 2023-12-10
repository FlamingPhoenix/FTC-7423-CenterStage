package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class TeleOp2 extends OpMode {
    double baseMaxDistance = 40;
    DcMotor fl, fr, bl, br, intake, liftl, liftr;
    Servo clawServo,arml,armr;
    DistanceSensor dm;
    IMU imu;
    Claw claw;
    Lift lift;
    ServoArm servoArm;
    armAssemblyTeleOp armAssembly;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    double oldDistanceFromBackdrop = baseMaxDistance;
    ElapsedTime elapsedTime;
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

        dm = hardwareMap.get(DistanceSensor.class,"dm");

        initAprilTag();

        elapsedTime.reset();

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
        //Calculate motor powers for field centric drive
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


        //Prevent crashing into backdrop and possibly descoring pixels
        double distanceFromBackdrop = dm.getDistance(DistanceUnit.INCH);
        double backdropApproachSpeed = (Math.max(oldDistanceFromBackdrop - distanceFromBackdrop, 0))/(96*elapsedTime.seconds());
        elapsedTime.reset(); //UNTESTED

        double maxDistance = baseMaxDistance * Math.pow(backdropApproachSpeed, 2);
        double motorPowerMultiplier = 1;

        telemetry.addData("maxDistance",maxDistance);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.metadata!=null){
                telemetry.addData("aprilTag","detected");
                telemetry.addData("id",detection.id);
                if ((detection.ftcPose.y < maxDistance) && detection.id >= 1 && detection.id <= 6) {
                    motorPowerMultiplier = Math.pow(backdropApproachSpeed, 2);
                }
            }
        }

        if(distanceFromBackdrop < maxDistance){
            oldDistanceFromBackdrop = dm.getDistance(DistanceUnit.INCH);
            motorPowerMultiplier = Math.pow(backdropApproachSpeed, 2);
        }

        oldDistanceFromBackdrop = Math.min(distanceFromBackdrop, baseMaxDistance);

        //Actually set the power of the motors
        fl.setPower(0.49*flp*motorPowerMultiplier);
        bl.setPower(0.49*blp*motorPowerMultiplier);
        fr.setPower(0.49*frp*motorPowerMultiplier);
        br.setPower(0.49*brp*motorPowerMultiplier);

        //Run armAssembly
        armAssembly.execute();

        //Run intake
        if(gamepad1.right_trigger > 0.1){
            intake.setPower(gamepad1.right_trigger/2);
        }else{
            intake.setPower(0);
        }

        telemetry.update();
    }

    private void initAprilTag(){
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }
}
