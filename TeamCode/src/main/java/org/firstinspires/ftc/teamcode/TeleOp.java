package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GlobalImu;

//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestTeleOp", group="Robot")
public class TeleOp extends OpMode {
    DcMotor fl, fr, bl, br, lift, intake;
    BNO055IMU imu;

    /*boolean goForHigh = false, goForMiddle = false, goForLow = false,
            goReleaseCone = false, processReleaseCone = false;
    int liftReleaseStartPos = 0;*/

    @Override
    public void init() {
        // Declare our motors
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        //TouchSensor touch = hardwareMap.touchSensor.get("touch");

        //DigitalChannel touch = hardwareMap.get(DigitalChannel.class, "touch");

        // set the digital channel to input.
        //touch.setMode(DigitalChannel.Mode.INPUT);

        //lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");


        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the right side motors
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


    }

    @Override
    public void loop() {
        //constant lift power
        lift.setPower(-0.15f);

        //different servo positions and lift power

        if (gamepad1.right_trigger > 0.5)
            intake.setPower(0.5f);

        else if (gamepad1.right_trigger < 0.5)
            intake.setPower(0.0f);

        /*if (gamepad2.b)
            lift.setPower(-0.5f);

        if (gamepad2.left_stick_y<-0.1f) { //Make sure grabber is closed when lift is going up
            lift.setPower(gamepad2.left_stick_y);
        }
        if (gamepad2.left_stick_y>0.1f) { //Drive is using the joystick to move lift down
            lift.setPower(0.5 * gamepad2.left_stick_y);
        }
        if (gamepad2.right_bumper && !goReleaseCone && !processReleaseCone && lift.getCurrentPosition()<-125){
            goReleaseCone = true;
        }
        float liftCurrentPos = lift.getCurrentPosition();

        telemetry.addData("Current Pos %d", liftCurrentPos);
        telemetry.update();
        if (gamepad1.left_bumper && gamepad1.right_bumper){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
        if (gamepad2.dpad_up) {
            goForHigh = true;
            goForMiddle = false;
            goForLow = false;
        }
        else if (gamepad2.dpad_right) {
            goForHigh = false;
            goForMiddle = true;
            goForLow = false;
        }
        else if (gamepad2.dpad_down) {
            goForHigh = false;
            goForMiddle = false;
            goForLow = true;
        }
        else if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
        {
            goForHigh = false;
            goForMiddle = false;
            goForLow = false;
        }

        if (goForHigh) {
            if (liftCurrentPos>-910){
                lift.setPower(-1f);
            }
            else{
                lift.setPower(-0.15f);
            }
        } else if (goForMiddle) {
            if (liftCurrentPos>-665){
                lift.setPower(-1f);
            }
            else if (liftCurrentPos<-690){
                lift.setPower(0.3f);
            }
            else{
                lift.setPower(-0.15f);
            }
        } else if (goForLow) {
            if (liftCurrentPos>-425){
                lift.setPower(-1f);
            }
            else if (liftCurrentPos<-455){
                lift.setPower(0.3f);
            }
            else{
                lift.setPower(-0.15f);
            }
        }else if (goReleaseCone){
            processReleaseCone = true;
            goReleaseCone = false;
            liftReleaseStartPos = lift.getCurrentPosition();
        } else if (processReleaseCone) {
            if (liftCurrentPos < (liftReleaseStartPos + 100))
                lift.setPower(0.5f);
            else {
                processReleaseCone = false;
                lift.setPower(-0.15f);
            }*/
        

        //math for field centric
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        while (GlobalImu.imuAngle+imu.getAngularOrientation().firstAngle>179 || GlobalImu.imuAngle+imu.getAngularOrientation().firstAngle<-179){
            if (GlobalImu.imuAngle+imu.getAngularOrientation().firstAngle>0){
                GlobalImu.imuAngle = GlobalImu.imuAngle-180;
            }
            if (GlobalImu.imuAngle+imu.getAngularOrientation().firstAngle<0){
                GlobalImu.imuAngle = GlobalImu.imuAngle+180;
            }
        }
        double botHeading = -(imu).getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flp = -(rotY + rotX + rx) / denominator;
        double blp = -(rotY - rotX + rx) / denominator;
        double frp = -(rotY - rotX - rx) / denominator;
        double brp = -(rotY + rotX - rx) / denominator;
            /*if (gamepad1.left_trigger>0.2){
                flp = ((1.2-gamepad1.left_trigger) *flp);
                blp = ((1.2-gamepad1.left_trigger) *blp);
                frp = ((1.2-gamepad1.left_trigger) *frp);
                brp = ((1.2-gamepad1.left_trigger)  *brp);
            }*/
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
    }
}