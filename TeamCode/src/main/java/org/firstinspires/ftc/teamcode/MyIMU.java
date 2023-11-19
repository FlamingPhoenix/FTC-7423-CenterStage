package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class MyIMU implements BNO055IMU {
    public Orientation startOrientation;
    private BNO055IMU myIMU;
    private Direction _direction;

    //Constructor
    public MyIMU(HardwareMap hardwareMap) {
        myIMU = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public boolean initialize(Parameters parameters) {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        boolean r = myIMU.initialize((parameters));
        reset(Direction.NONE);

        return r;
    }

    public void reset(Direction turnDirection) {
        startOrientation = myIMU.getAngularOrientation();
        _direction = turnDirection;
    }

    public float getAdjustedAngle() {

        float currentAngle = myIMU.getAngularOrientation().firstAngle;

        if (startOrientation.firstAngle > 0 && currentAngle < 0 && _direction == Direction.COUNTERCLOCKWISE) {
            currentAngle += 360;
        } else if(startOrientation.firstAngle < 0 && currentAngle > 0 && _direction == Direction.CLOCKWISE) {
            currentAngle -= 360;
        }

        return currentAngle;
    }

    @NonNull
    @Override
    public Parameters getParameters() {
        return myIMU.getParameters();
    }

    @Override
    public void close() {
        myIMU.close();
    }

    @Override
    public Orientation getAngularOrientation() {
        return myIMU.getAngularOrientation();
    }

    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return myIMU.getAngularOrientation(reference, order, angleUnit);
    }

    @Override
    public Acceleration getOverallAcceleration() {
        return myIMU.getOverallAcceleration();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return myIMU.getAngularVelocity();
    } // add myIMU Nov 1 to return.

    @Override
    public Acceleration getLinearAcceleration() {
        return null;
    }

    @Override
    public Acceleration getGravity() {
        return null;
    }

    @Override
    public Temperature getTemperature() {
        return null;
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return null;
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        return null;
    }

    @Override
    public Position getPosition() {
        return myIMU.getPosition();
    }

    @Override
    public Velocity getVelocity() {
        return myIMU.getVelocity();
    }

    @Override
    public Acceleration getAcceleration() {
        return myIMU.getAcceleration();
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
        myIMU.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval);
    }

    @Override
    public void stopAccelerationIntegration() {
        myIMU.stopAccelerationIntegration();
    }

    @Override
    public SystemStatus getSystemStatus() {
        return myIMU.getSystemStatus();
    }

    @Override
    public SystemError getSystemError() {
        return myIMU.getSystemError();
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return myIMU.getCalibrationStatus();
    }

    @Override
    public boolean isSystemCalibrated() {
        return myIMU.isSystemCalibrated();
    }

    @Override
    public boolean isGyroCalibrated() {
        return myIMU.isGyroCalibrated();
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return myIMU.isAccelerometerCalibrated();
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return myIMU.isMagnetometerCalibrated();
    }

    @Override
    public CalibrationData readCalibrationData() {
        return myIMU.readCalibrationData();
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {

    }

    @Override
    public byte read8(Register register) {
        return 0;
    }

    @Override
    public byte[] read(Register register, int cb) {
        return new byte[0];
    }

    @Override
    public void write8(Register register, int bVal) {

    }

    @Override
    public void write(Register register, byte[] data) {

    }
}