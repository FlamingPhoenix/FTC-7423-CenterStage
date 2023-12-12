package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class ServoArm { //12-3: still in progress
    Servo arml, armr;
    double maxPos = 1; //extended
    double minPos = 0; //retracted
    public ServoArm(@NotNull Servo arml, @NotNull Servo armr, double maxPos, double minPos){
        this.arml = arml;
        this.armr = armr;
        arml.setPosition(minPos);
        armr.setPosition(1-minPos);
    }
    public void setPosition(double pos){
     arml.setPosition(pos);
     armr.setPosition(1-pos);
    }
    public void extend(){
        arml.setPosition(maxPos);
        armr.setPosition(1-maxPos);
    }
    public void retract(){
        arml.setPosition(minPos);
        armr.setPosition(1-minPos);
    }
}
