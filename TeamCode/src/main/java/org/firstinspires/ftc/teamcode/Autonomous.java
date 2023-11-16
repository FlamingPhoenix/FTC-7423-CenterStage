package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear Opmode")
public class Autonomous extends LinearOpMode {

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor fl,fr,bl,br;

    boolean hastarget = false;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    final double DESIRED_DISTANCE = 3.0; //  this is how close the camera should get to the target (inches)
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";
    private TfodProcessor tfod;
    double[] xposs = {-40.96,-34.96,-28.96,-34.96};
    String[] poss = {"left","center","right","none"};
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initTfod();
        //find team prop
        //set variable to 0 - left   1 - center   2 - right 3 - none
        int teamProp = getPos();
        telemetry.addData("side",poss[teamProp]);
        TrajectorySequence go2backdrop = drive.trajectorySequenceBuilder(new Pose2d(-63.00, 12.00, Math.toRadians(0.00)))
                .splineTo(new Vector2d(xposs[teamProp], 49.01), Math.toRadians(90.00))
                .build();
        TrajectorySequence loop = drive.trajectorySequenceBuilder(go2backdrop.end()).lineToConstantHeading(new Vector2d(-35.30, -4.75))
                .lineToSplineHeading(new Pose2d(-36.05, -56.35, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-13.51, -36.05), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-12.76, 12.95), Math.toRadians(91.43))
                .splineToConstantHeading(new Vector2d(-36.05, 49.46), Math.toRadians(90.00))
                .build();
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        setManualExposure(6,250);
        initAprilTag();
        waitForStart();
        //start of autonomous
        drive.followTrajectorySequence(go2backdrop);

        while (opModeIsActive()) {
            telemetry.addData("side",poss[teamProp]);
            telemetry.update();

        }
    }

    public void getToApril(int teamProp){
        int detectionId = 0;
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;
        targetFound = false;
        hastarget = false;
        desiredTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) ){
                if((detection.id == teamProp) || (detection.id == teamProp + 3)) {
                    telemetry.addData("Target Found", "Tag ID %d is %s", detection.id, detection.ftcPose.toString());
                    targetFound = true;

                    desiredTag = detection;
                    detectionId = detection.id;
                    break;  // don't look any further.
                }else{
                    desiredTag = detection;
                }
                hastarget = true;
                detectionId = detection.id;
                telemetry.addData("current tag", detectionId);
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        if (targetFound) {
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        }else if(hastarget){
            drive = 0;
            turn = 0;
            strafe = strafeDirectionToApril(teamProp,detectionId)*0.3;
        }
        else{
            drive = 0;
            turn = 0;
            strafe = 0;
            telemetry.addData("No Target", "No AprilTag target is detected\n");
        }
        moveRobot(drive, strafe, turn);
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

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private double strafeDirectionToApril(int targetTag, int currentTag){
            if(targetTag<currentTag) {
                return 1;
            }else if(targetTag>currentTag) {
                return -1;
            }else{
                return 0;//we are at equilibrium
            }
    }
    private void initTfod() {

        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_ASSET).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 640));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.7f);
    }
    private int getDetection() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        int pos = 3;
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size()>0){
            Recognition recognition = currentRecognitions.get(0);
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            //double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if(x>=0 && x<=213){
                pos = 0;
            }else if(x>213 && x<=426){
                pos = 1;
            }else if(x>426 && x<=640){
                pos = 2;
            }
            telemetry.addData("side",poss[pos]);
        }   // end for() loop
        return pos;
    }   // end method telemetryTfod()

    private int getPos(){
        int[] counts = new int[4];
        for(int i = 0; i<10; i++){
            counts[getDetection()]++;  //get 10 detections & note the outputs
            sleep(20);
        }
        int max = 0;
        int maxIndex = 0;
        for(int i = 0; i<4; i++){
            if(counts[i]>max){
                max = counts[i]; // get the max count from the detections -- this is the most common output
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}
