package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MyIMU implements IMU {
    public Orientation startOrientation;
    private IMU myIMU;
    private Direction _direction;
    private AngleUnit _angleUnit;

    //Constructor
    public MyIMU(HardwareMap hardwareMap, AngleUnit angleUnit) {
        myIMU = hardwareMap.get(IMU.class, "imu");
        _angleUnit = angleUnit;
    }
    public MyIMU(HardwareMap hardwareMap) {
        myIMU = hardwareMap.get(IMU.class, "imu");
        _angleUnit = AngleUnit.DEGREES;
    }
    public boolean initialize(Parameters parameters) {

        boolean r = myIMU.initialize((parameters));
        resetYaw();

        return r;
    }

    @Override
    public void resetYaw() {
        myIMU.resetYaw();
    }

    public float getAdjustedAngle(){
        float currentAngle = (float)myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (startOrientation.firstAngle > 0 && currentAngle < 0 && _direction == Direction.COUNTERCLOCKWISE) {
            currentAngle += 360;
        } else if(startOrientation.firstAngle < 0 && currentAngle > 0 && _direction == Direction.CLOCKWISE) {
            currentAngle -= 360;
        }

        return currentAngle;
    }
    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return myIMU.getRobotYawPitchRollAngles();
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return myIMU.getRobotOrientation(reference,order,angleUnit);
    }
    public Orientation getAngularOrientation(){
        return myIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, _angleUnit);
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return myIMU.getRobotOrientationAsQuaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return getRobotAngularVelocity(_angleUnit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return myIMU.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return myIMU.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return myIMU.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return myIMU.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        myIMU.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        myIMU.close();
    }
}