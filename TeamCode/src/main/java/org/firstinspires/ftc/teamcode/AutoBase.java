package org.firstinspires.ftc.teamcode;

import android.provider.CalendarContract;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor lift;
    DistanceSensor distanceLeft, distanceMid, distanceRight;
    NormalizedColorSensor colorSensorLeft;
    NormalizedColorSensor colorSensorRight;
    NormalizedColorSensor colorSensorGrabber;
    NormalizedRGBA colorsLeft;
    NormalizedRGBA colorsRight;
    NormalizedRGBA colorsGrabber;


    Servo intakeLeft, intakeRight; //intakeLeft is not used because one servo is enough
    Servo vbarLeft, vbarRight;
    Servo grabber;

    float pos = 0.5f;
    float vposR = 0.69f;
    float vposL = 0.7f;
    float fpos = 0.2f;

    public Servo pivotLeft;
    public Servo pivotRight;

    public DigitalChannel touch;
    public float PPR = 537.7f; //537.7 for actual robot; 1120 for programming bot

    public float maxEncoderPulley = 420f;

    public float diameter = 4;

    public MyIMU imu;


    public float startHeading;

    public int currentStage, currentPosition;

    //BNO055IMU imu;

    public void initialize (){
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");



        // set the digital channel to input.
        //touch.setMode(DigitalChannel.Mode.INPUT);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo grabber = hardwareMap.servo.get("grabber");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "dl");
        distanceRight = hardwareMap.get(DistanceSensor.class, "dr");
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "csl");
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "csr");
        colorSensorGrabber = hardwareMap.get(NormalizedColorSensor.class, "gcs");
        //imu = new MyIMU(hardwareMap);
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //imu.initialize(parameters);
        // Without this, data retrieving from the IMU throws an exception
        //imu.initialize(parameters);
    }


    public void intake(){
        colorsGrabber = colorSensorGrabber.getNormalizedColors();
        if (colorsGrabber.red >0.002 ^ colorsGrabber.blue>0.002)
            grabber.setPosition(0.6f);
    }
    public void outtake(){
        grabber.setPosition(1f);
    }

    public void stripeCorrection(){
        colorsRight = colorSensorRight.getNormalizedColors();
        colorsLeft = colorSensorLeft.getNormalizedColors();
        if((colorsRight.red >0.004 && colorsLeft.red>0.004)){
            telemetry.addLine("YIPPEE");
        }
        if((colorsRight.blue >0.004 && colorsLeft.blue>0.004)){
            telemetry.addLine("YIPPEE");
        }
        telemetry.update();
        while ((!(colorsRight.red>0.004) ^ !(colorsLeft.red>0.004))){
            if (colorsRight.red<0.004){
                Strafe(0.175f, 0.35f, Direction.RIGHT);
            }
            if((colorsRight.red >0.004 && colorsLeft.red>0.004)){
                telemetry.addLine("YIPPEE");
                break;
            }
            if (colorsLeft.red<0.004){
                Strafe(0.175f, 0.35f, Direction.LEFT);
            }

            colorsRight = colorSensorRight.getNormalizedColors();
            colorsLeft = colorSensorLeft.getNormalizedColors();


        }
        while ((!(colorsRight.blue>0.004) ^ !(colorsLeft.blue>0.004))){
            if (colorsRight.blue<0.004){
                Strafe(0.3f, 1.5f, Direction.RIGHT);
            }
            if (colorsLeft.blue<0.004){
                Strafe(0.3f, 1.5f, Direction.LEFT);
            }
            colorsRight = colorSensorRight.getNormalizedColors();
            colorsLeft = colorSensorLeft.getNormalizedColors();


        }
    }
    public void distanceTune() {
        double d1 = distanceLeft.getDistance(DistanceUnit.CM);
        double d2 = distanceRight.getDistance(DistanceUnit.CM);
        if ((lift.getCurrentPosition()>1)){
            while(!(d1<12 && d1>3.5) ^ !(d2<12 && d2>3.5)){
                if(!(d1<12 && d1>3.5)){
                    Strafe(0.25f, 0.5f, Direction.LEFT);
                }
                if(!(d2<12 && d2>3.5)){
                    Strafe(0.25f, 0.5f, Direction.RIGHT);
                }
                d1 = distanceLeft.getDistance(DistanceUnit.CM);
                d2 = distanceRight.getDistance(DistanceUnit.CM);
            }
            outtake();

        }

    }






    public void DistanceStrafe(float power)  {

        double d1 = distanceLeft.getDistance(DistanceUnit.INCH);
        double d2 = distanceRight.getDistance(DistanceUnit.INCH);
        if (((d2<12 && d2>1.5) && !(d1<12 && d1>1.5)))  {
            while (((d2<12 && d2>1.5) && !(d1<12 && d1>1.5) && opModeIsActive())) {
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
                d1 = distanceLeft.getDistance(DistanceUnit.INCH);
                d2 = distanceRight.getDistance(DistanceUnit.INCH);
            }
        } else if (!(d2<12 && d2>1.5) && (d1<12 && d1>1.5)) {
            while (!(d2<12 && d2>1.5) && (d1<12 && d1>1.5) && opModeIsActive()) {
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                d1 = distanceLeft.getDistance(DistanceUnit.INCH);
                d2 = distanceRight.getDistance(DistanceUnit.INCH);
            }
        }
        StopAll();
    }
    public void DistanceDrive (float power){
        double d1 = distanceLeft.getDistance(DistanceUnit.INCH);
        double d2 = distanceRight.getDistance(DistanceUnit.INCH);

        if ((d1+d2)<50){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }
    }

    public void DistanceScore (float power) {
        double d1 = distanceLeft.getDistance(DistanceUnit.INCH);
        double d2 = distanceRight.getDistance(DistanceUnit.INCH);
        long startTime = System.currentTimeMillis();
        long elapsedTime = 0;
        while (d1 > 7.5 || d2 > 7.5) {
            DistanceStrafe(2 * power);
            DistanceDrive(power);
            d1 = distanceLeft.getDistance(DistanceUnit.INCH);
            d2 = distanceRight.getDistance(DistanceUnit.INCH);
            telemetry.addData("d1 %f", d1);
            telemetry.addData("d2 %f", d2);
            telemetry.update();
            power = 0.99f*power;
            elapsedTime = System.currentTimeMillis() - startTime;
            if (elapsedTime >= 2000) { // 10 seconds timeout
                break;
            }
        }
        StopAll();
    }

    public void stripeStrafe(float power) {

        double targetValue = 0.004;
        double kp = 0.5;
        double error, correction;

        colorsRight = colorSensorRight.getNormalizedColors();
        colorsLeft = colorSensorLeft.getNormalizedColors();

        while ((colorsRight.red > targetValue && colorsLeft.red < targetValue) ||
                (colorsRight.blue > targetValue && colorsLeft.blue < targetValue) && opModeIsActive()) {

            error = (colorsRight.red + colorsRight.blue) - (colorsLeft.red + colorsLeft.blue);
            correction = kp * error;

            fl.setPower(power - correction);
            fr.setPower(-power + correction);
            bl.setPower(-power - correction);
            br.setPower(power + correction);

            colorsRight = colorSensorRight.getNormalizedColors();
            colorsLeft = colorSensorLeft.getNormalizedColors();
        }

        StopAll();
    }

    public void StripeStrafe(float power)  {

        colorsRight = colorSensorRight.getNormalizedColors();
        colorsLeft = colorSensorLeft.getNormalizedColors();

        if (colorsRight.red >0.004 && colorsLeft.red  <0.004) {
            while (colorsRight.red >0.004 && colorsLeft.red  <0.004 && opModeIsActive()) {
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
                colorsRight = colorSensorRight.getNormalizedColors();
                colorsLeft = colorSensorLeft.getNormalizedColors();
            }
        } else {
            while (colorsRight.red <0.004 && colorsLeft.red > 0.004 && opModeIsActive()) {
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                colorsRight = colorSensorRight.getNormalizedColors();
                colorsLeft = colorSensorLeft.getNormalizedColors();
            }
        }
        if (colorsRight.blue >0.004 && colorsLeft.blue  <0.004) {
            while (colorsRight.blue >0.004 && colorsLeft.blue  <0.004 && opModeIsActive()) {
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
                colorsRight = colorSensorRight.getNormalizedColors();
                colorsLeft = colorSensorLeft.getNormalizedColors();
            }
        } else {
            while (colorsRight.blue <0.004 && colorsLeft.blue > 0.004 && opModeIsActive()) {
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                colorsRight = colorSensorRight.getNormalizedColors();
                colorsLeft = colorSensorLeft.getNormalizedColors();
            }
        }


        StopAll();
    }

    public void StripeAlign(float power){
        NormalizedRGBA colors = colorSensorGrabber.getNormalizedColors();
        while (colors.red <0.004 && colors.blue <0.004) {

            StripeStrafe(0.18f);
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
            colors = colorSensorGrabber.getNormalizedColors();

        }
        Drive(1);


    }

    public void setStage(int coneNumber){
        float currentPos = lift.getCurrentPosition();
        if (coneNumber == 5){
            while (currentPos <-140){
                lift.setPower(0.3f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }

        }
        else if (coneNumber == 4){
            while (currentPos <-105){
                lift.setPower(0.3f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }

        }
        else if (coneNumber == 3){
            while (currentPos <-70){
                lift.setPower(0.3f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }

        }
        else if (coneNumber == 2){
            while (currentPos <-35){
                lift.setPower(0.3f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }

        }
        else if (coneNumber == 10){
            while (currentPos >-950){
                lift.setPower(-1f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }

        }


        else{
            while (currentPos <-5){
                lift.setPower(1f);
                currentPos = lift.getCurrentPosition();
                telemetry.addData("Current Pos %d", currentPos);
                telemetry.update();
            }
        }
        lift.setPower(-0.15f);
    }

    public void StopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void Drive(double distance){
        double x = (PPR * distance)/(diameter * (float)Math.PI);

        long targetEncoderValue = Math.round(x);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && opModeIsActive()) {
            currentPosition = Math.abs(fr.getCurrentPosition());

            fl.setPower(0.2f);
            fr.setPower(0.2f);
            bl.setPower(0.2f);
            br.setPower(0.2f);
            telemetry.addData("Current Pos %d", currentPosition);
            telemetry.addData("target", targetEncoderValue);
            telemetry.update();
        }
        StopAll();

    }

    public void Drive (float distance, Direction d) {
        //tune the program by starting with Kd at 0
        //then increase Kd until the robot will approach the position slowly with little overshoot

        float Kp = 0.1f;
        float Ki = 0.01f;

        float Kd = 0; //tune this

        float reference = Math.round((PPR * distance)/(diameter * (float)Math.PI));
        if (d == Direction.BACKWARD)
            reference = -reference;
        double integralSum = 0;
        float lastError = 0;
        float out;
        float firstAngle = imu.getAdjustedAngle();
        float adjustmentAngle = imu.getAdjustedAngle();
        float currentAngle;
        float adjustment = 1;//need to tune
        ElapsedTime timer = new ElapsedTime();
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < reference && opModeIsActive()) {
            currentPosition = Math.abs(bl.getCurrentPosition());
            float error = reference - currentPosition;
            double derivative = (error - lastError)/timer.seconds();//derivatives are instantaneous rates of change; this is a difference quotient - essentially just a slope formula
            integralSum = integralSum + (error*timer.seconds());//integral through riemann sums - approximating area under the curve with a bunch of rectangles
            out = (float)((Kp*error)+(Ki*error)+(Kd*derivative));

            if (Math.abs(firstAngle) >= 178) {//any number (1,179) works here
                currentAngle = imu.getAdjustedAngle() - adjustmentAngle;
                firstAngle = 0;
            } else
                currentAngle = imu.getAdjustedAngle();


            if (Math.abs(currentAngle - firstAngle) > 1) { //error > 1 degree
                if ((currentAngle - firstAngle) < 0)
                    setMaxPower((out + adjustment), out, (out + adjustment), out);
                else
                    setMaxPower(out, (out + adjustment), out, (out + adjustment));
            } else
                setMaxPower(out, out, out, out);

            lastError = error;



            timer.reset();
        }

        StopAll();

    }
    public void Drive (float power, float distance, Direction d) {
        float x = (PPR * distance)/(diameter * (float)Math.PI);

        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && opModeIsActive()) {
            currentPosition = Math.abs(fl.getCurrentPosition());
            if (d == Direction.FORWARD) {
                fl.setPower(-power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(-power);


            }
            if (d == Direction.BACKWARD) {
                fl.setPower(power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(power);

            }
        }

        StopAll();
        telemetry.addLine("DONE");
        telemetry.update();

    }

    public void StopAllWheels(){
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }



    public void Turn (float power, double angle, Direction turnDirection, MyIMU imu) {


        Orientation startOrientation = imu.getAngularOrientation();

        double targetangle;
        double currentangle;

        imu.reset(turnDirection);
        if (turnDirection == Direction.COUNTERCLOCKWISE) {

            targetangle = startOrientation.firstAngle + angle;
            currentangle = startOrientation.firstAngle;

            while (currentangle < targetangle && opModeIsActive()) {

                currentangle = imu.getAdjustedAngle();

                Log.i("[phoenix:angleInfo]", String.format("startingAngle = %f, targetAngle = %f, currentAngle = %f", startOrientation.firstAngle, targetangle, currentangle));

                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);

            }
        }

        else {

            targetangle = startOrientation.firstAngle - angle;
            currentangle = startOrientation.firstAngle;

            while (currentangle > targetangle && opModeIsActive()) {

                currentangle = imu.getAdjustedAngle();

                this.telemetry.addData("CurrentAngle =: %f", currentangle);
                this.telemetry.update();

                fl.setPower(-power);
                bl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);


            }
        }

        StopAll();

    }

    public void Strafe(float power, float distance, Direction d) {
        float x = (PPR * (2 * distance))/(diameter * (float)Math.PI);

        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        if (d == Direction.LEFT) {
            while (currentPosition < targetEncoderValue && opModeIsActive()) {
                currentPosition = Math.abs(fl.getCurrentPosition());
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
            }
        } else {
            while (currentPosition < targetEncoderValue && opModeIsActive()) {
                currentPosition = Math.abs(fl.getCurrentPosition());
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
            }
        }

        StopAll();
    }

    public void Strafe(float power, float distance, Direction d, float maxTime) {
        float x = (PPR * (2 * distance))/(diameter * (float)Math.PI);

        int targetEncoderValue = Math.round(x);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        maxTime += System.currentTimeMillis();
        float currentTime = System.currentTimeMillis();

        if (d == Direction.LEFT) {
            while (currentPosition < targetEncoderValue && opModeIsActive() && maxTime - currentTime > 0) {
                currentPosition = Math.abs(bl.getCurrentPosition());
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                currentTime = System.currentTimeMillis();
            }
        } else {
            while (currentPosition < targetEncoderValue && opModeIsActive() && maxTime - currentTime > 0) {
                currentPosition = Math.abs(bl.getCurrentPosition());
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
                currentTime = System.currentTimeMillis();
            }
        }

        StopAll();
    }

    public float Max(float f1, float f2, float f3, float f4) {
        f1 = Math.abs(f1);
        f2 = Math.abs(f2);
        f3 = Math.abs(f3);
        f4 = Math.abs(f4);


        if(f1>=f2 && f1>=f3 && f1>=f4) return f1;
        if(f2>=f1 && f2>=f3 && f2>=f4) return f2;
        if(f3>=f1 && f1>=f2 && f1>=f4) return f3;
        return f4;



    }

    public void DriveHeading(float power, float distance, float heading, float adjust, Direction turnDirection){

        power = Math.abs(power);

        imu.reset(Direction.NONE);

        float currentHeading = imu.getAdjustedAngle();

        float x = (PPR * distance)/(diameter * (float)Math.PI);

        int targetEncoderValue = Math.round(x);

        Log.i("[phoenix:startValues]", String.format("Heading: %f; Target Encoder: %d", heading, targetEncoderValue));


        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && opModeIsActive()) {
            Log.i("[phoenix:currentHead]", String.format("Current Heading: %f; heading: %f", currentHeading, heading));
            float adjustmentPower = 0;
            currentHeading = imu.getAdjustedAngle();

            if (currentHeading > 0 && heading > 0) {
                if (currentHeading - heading > 1) {
                    adjustmentPower = adjust;
                } else if (currentHeading - heading < -1) {
                    adjustmentPower = -adjust;
                }
            } else if (currentHeading < 0 && heading < 0) {
                if (currentHeading - heading > 1) {
                    adjustmentPower = adjust;
                } else if (currentHeading - heading < -1) {
                    adjustmentPower = -adjust;
                }
            } else if (currentHeading < 0 && heading > 0) {
                if (currentHeading - heading < -1) {
                    adjustmentPower = -adjust;
                }
            } else {
                if (currentHeading - heading > 1) {
                    adjustmentPower = adjust;
                } else if (currentHeading - heading < -1) {
                    adjustmentPower = -adjust;
                }
            }

            if (turnDirection == Direction.BACKWARD)
                adjustmentPower = adjustmentPower * -1;

            currentPosition = Math.abs(bl.getCurrentPosition());

            float frontRight = power;
            float frontLeft = power;
            float backRight = power;
            float backLeft = power;

            if (adjustmentPower > 0) {  //positive means give more power to the left
                frontLeft = (power + adjustmentPower);
                backLeft = (power + adjustmentPower);
            }
            else if (adjustmentPower < 0) {
                frontRight = (power - adjustmentPower);
                backRight = (power - adjustmentPower);
            }

            if (turnDirection == Direction.FORWARD) {
                setMaxPower(frontLeft, frontRight, backLeft, backRight);
            } else if (turnDirection == Direction.BACKWARD) {
                setMaxPower(-frontLeft, -frontRight, -backLeft, -backRight);
            }


            Log.i("[phoenix:wheelPowers]", String.format("frontLeft: %f; frontRight: %f; backLeft: %f; backRight: %f", frontLeft, frontRight, backLeft, backRight));
        }

        StopAll();

    }

    public void setMaxPower(float flp, float frp, float blp, float brp) {
        float max = Max(Math.abs(flp), Math.abs(frp), Math.abs(blp), Math.abs(brp));

        if (max > 1) {
            fl.setPower(flp/max);
            fr.setPower(frp/max);
            bl.setPower(blp/max);
            br.setPower(brp/max);
        } else {
            fl.setPower(flp);
            fr.setPower(frp);
            bl.setPower(blp);
            br.setPower(brp);
        }
    }
    public void moveTurret(float amount){

    }



    public void StrafeUntilHeading(float power, float multiplier, float heading, float distance, Direction d) {

        float x = (PPR * (2 * distance))/(diameter * (float)Math.PI);

        int targetEncoderValue = Math.round(x);
        float powerAdjustRatio = 1 + Math.abs(multiplier);

        float flp, frp, blp, brp;

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        if (d == Direction.LEFT) {
            while (currentPosition < targetEncoderValue && opModeIsActive()) {
                currentPosition = Math.abs(bl.getCurrentPosition());
                float currentHeading = imu.getAdjustedAngle();
                float angleDiff = Math.abs(currentPosition-heading);
                if (angleDiff > 20) {
                    powerAdjustRatio = 1+Math.abs(multiplier);
                } else if (angleDiff > 5) {
                    powerAdjustRatio = 1+Math.abs(multiplier*0.5f);
                } else {
                    powerAdjustRatio = 1+Math.abs(multiplier*0.1f);
                }

                flp = -power;
                frp = power;
                blp = power;
                brp = -power;

                if (currentHeading * heading < 0 && currentHeading > 0) {
                    heading += 360;
                } if (currentHeading * heading < 0  && currentHeading < 0) {
                    heading -= 360;
                }

                if (currentHeading < heading) {
                    blp /= powerAdjustRatio;
                    brp /= powerAdjustRatio;
                }
                else if (currentHeading > heading) {
                    flp /= powerAdjustRatio;
                    frp /= powerAdjustRatio;

                }

                setMaxPower(flp, frp, blp, brp);
                Log.i("[phoenix:strafeHeading]", String.format("currentHeading: %f, heading: %f, flp: %f, frp: %f, blp: %f, brp: %f", currentHeading, heading, flp, frp, blp, brp));
            }
        } else {
            while (currentPosition < targetEncoderValue && opModeIsActive()) {
                currentPosition = Math.abs(bl.getCurrentPosition());
                float currentHeading = imu.getAdjustedAngle();

                flp = power;
                frp = -power;
                blp = -power;
                brp = power;

                if (currentHeading * heading < 0 && currentHeading > 0) {
                    heading += 360;
                } if (currentHeading * heading < 0  && currentHeading < 0) {
                    heading -= 360;
                }

                if (currentHeading < heading) {
                    flp /= powerAdjustRatio;
                    frp /= powerAdjustRatio;
                }
                else if (currentHeading > heading) {
                    blp /= powerAdjustRatio;
                    brp /= powerAdjustRatio;
                }

                setMaxPower(flp, frp, blp, brp);
                Log.i("[phoenix:strafeHeading]", String.format("currentHeading: %f, heading: %f, flp: %f, frp: %f, blp: %f, brp: %f", currentHeading, heading, flp, frp, blp, brp));
            }
        }

        StopAll();
    }

    /*public void MovePulley (float power, int stage) {

        // stage can be 0, 1, 2 depending on the shipping hub level

        int targetEncoderValue = 0;

        if (stage == 1) {
            targetEncoderValue = 200;
        } else if (stage == 2) {
            targetEncoderValue = 340;
        }

        pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (currentStage > stage) {
            while (currentPosition > targetEncoderValue && opModeIsActive()) {
                vposR = 0.75f;
                vbarRight.setPosition(vposR);
                sleep(1500);
                currentPosition = pulley.getCurrentPosition();
                Globals.pulleyEncoder = currentPosition;
                pulley.setPower(-power);
                pulley2.setPower(-power);
                Log.i("[phoenix:pulleyInfo]", String.format("currentPulley = %d", currentPosition));
            }
        } else if (currentStage < stage) {
            while (currentPosition < targetEncoderValue && opModeIsActive()) {
                currentPosition = pulley.getCurrentPosition();
                Globals.pulleyEncoder = currentPosition;
                pulley.setPower(power);
                pulley2.setPower(power);
                Log.i("[phoenix:pulleyInfo]", String.format("currentPulley = %d", currentPosition));
            }
            vposR = 0.2f;
            vbarRight.setPosition(vposR);
        }

        float backgroundPower = 0;

        currentStage = stage;

        // w/o lift attachment
        if (currentStage == 1) {
            backgroundPower = 0.1f;
        } else if (currentStage == 2) {
            backgroundPower = 0.1f;
        }

        pulley.setPower(backgroundPower);
        pulley2.setPower(backgroundPower);

    }

    public void OnStart(int duck) {
        switch (duck) {
            case 0:
                break;
            case 1:
                break;//need to test code before actually tuning this auto version to work
            case 2:
                Drive(0.5f, 5, Direction.BACKWARD);

                finger.setPosition(0.3f);
                vbarRight.setPosition(0.78f);
                while (pulley.getCurrentPosition() < 85) {
                    pulley.setPower(0.8f);
                    pulley2.setPower(0.8f);
                }
                pulley.setPower(0);
                pulley2.setPower(0);
                intakeRight.setPosition(0.71f);
                sweeper.setPower(0.7f);
                sleep(1000);
                while (pulley.getCurrentPosition() > 0) {
                    pulley.setPower(-0.5f);
                    pulley2.setPower(-0.5f);
                }
                pulley.setPower(0);
                pulley2.setPower(0);
                sweeper.setPower(0);
                vbarRight.setPosition(0.8f);
                sleep(100);
                finger.setPosition(0.60);

        }
    }

    public void Intake(float power){
        double intakeTime;
        double clampTime;

        int targetTime = 3000;

        sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sweeper.setPower(power);

        //Shake(0.5f, 10);

//        sleep(targetTime);
        sleep(1000);
        sweeper.setPower(0);

        Strafe(0.5f, 1, Direction.LEFT);

        DriveHeading(0.5f, 40, startHeading - 90, 0.3f, Direction.FORWARD);//can maybe seperate into two different functions here

        intakeRight.setPosition(0.4);
//        vbarRight.setPosition(0.69);
        finger.setPosition(0.1);
        sweeper.setPower(0.8);

        sleep(1000);

        sweeper.setPower(0);
        intakeRight.setPosition(0.7);

        sleep(1000);

        vbarRight.setPosition(0.79);
        finger.setPosition(0.5);
    } */
    public void Shake(float power, int shakes){
        int neg = 1;
        for (int i = 0; i < shakes; i++){
            fl.setPower(power * neg);
            bl.setPower(power * neg);
            fr.setPower(-power * neg);
            br.setPower(-power * neg);

            sleep(100);
            neg *= -1;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }



    public void velocityControl (float velocity, int totalTime, DcMotor dcmotor) { //controls velocity of robot in inches/seconds

        float out = 0;

        float countsVelocity = Math.round((PPR * velocity)/(diameter * (float)Math.PI));

        ElapsedTime timer = new ElapsedTime();

        dcmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int currentPosition;
        int lastPosition = 0;

        float adjustment = 0.1f;

        while (time < totalTime && opModeIsActive()) {

            currentPosition = Math.abs(dcmotor.getCurrentPosition());

            double derivative = (currentPosition - lastPosition)/timer.seconds();

            lastPosition = currentPosition;

            if (derivative < countsVelocity)
                out = out + adjustment;

            dcmotor.setPower(out);

            time = time + timer.seconds();

            timer.reset();
        }

        StopAll();

    }
}