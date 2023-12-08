package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class FieldCentric extends OpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    DcMotor fl, fr, bl, br, intake,liftl,liftr;
    Servo arml,armr, claw;
    private AprilTagProcessor aprilTag;
    DistanceSensor dl,dr;
    private VisionPortal visionPortal;
    public double motorPowerMultiplier = 1;
    double maxDistance;
    double servoPosition = 0;
    IMU imu;
    @Override
    public void init(){
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        intake = hardwareMap.dcMotor.get("intake");

        claw = hardwareMap.servo.get("claw");
        liftr = hardwareMap.dcMotor.get("liftr");
        liftl = hardwareMap.dcMotor.get("liftl");
        //arml = hardwareMap.servo.get("arml");
        //armr = hardwareMap.servo.get("armr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        //dl = hardwareMap.get(DistanceSensor.class,"dl");
        //dr = hardwareMap.get(DistanceSensor.class,"dr");
        double baseMaxDistance = 20;

        //brake
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        //BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        //BHI260IMU.Parameters paramaters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        //imu.initialize(paramaters);
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }
    @Override
    public void loop(){
        /*maxDistance = baseMaxDistance * Math.pow(1-gamepad1.left_stick_y,2)/3;
            telemetry.addData("maxDistance",maxDistance);
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if(detection.metadata!=null){
                    telemetry.addData("aprilTag","detected");
                    telemetry.addData("id",detection.id);
                    if ((dr.getDistance(DistanceUnit.INCH) < maxDistance)||((detection.ftcPose.y < maxDistance) && (detection.id == 1 || detection.id == 2 || detection.id == 3 || detection.id == 4 || detection.id == 5 || detection.id == 6)))

                        if(gamepad1.left_stick_y>0.1){
                            motorPowerMultiplier = 0.4;
                        }
                        else{
                            motorPowerMultiplier = 0.3;
                        } else {
                        motorPowerMultiplier = 1;
                    }
                }
            }
            if((dr.getDistance(DistanceUnit.INCH) < maxDistance)){

                if(gamepad1.left_stick_y>0.1){
                    motorPowerMultiplier = 0.4;
                }
                else{
                    motorPowerMultiplier = 0.3;
                }
            }
            else{
                motorPowerMultiplier = 1;
            }*/
        if(gamepad1.options){
            imu.resetYaw();
        }
        if(gamepad2.left_stick_y>0.1){
            liftl.setPower(gamepad2.left_stick_y*0.5);
            liftr.setPower(gamepad2.left_stick_y*0.5);
        } else{
            liftl.setPower(0);
            liftr.setPower(0);
        }
        if (gamepad1.right_bumper) {
            intake.setPower(0.4);
        }else if(gamepad1.left_bumper){
            intake.setPower(-0.28);
        }else{
            intake.setPower(0);
        }
        if(gamepad1.x){
            imu.resetYaw();
        }
        if(gamepad2.a){
            claw.setPosition(0.36);
        }
        if(gamepad2.b){
            claw.setPosition(0.47);
        }
        motorPowerMultiplier = 1;
        double y = gamepad1.left_stick_y*motorPowerMultiplier; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//might be degrees
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


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



        telemetry.update();

        fl.setPower(0.74*flp);
        bl.setPower(0.74*blp);
        fr.setPower(0.74*frp);
        br.setPower(0.74*brp);
    }
    private void initAprilTag(){
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        fl.setPower(leftFrontPower);
        fr.setPower(rightFrontPower);
        bl.setPower(leftBackPower);
        br.setPower(rightBackPower);
    }

}
