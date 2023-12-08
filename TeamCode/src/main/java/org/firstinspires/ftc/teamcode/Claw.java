package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class Claw {
    private Servo claw;

    private double[] posValues;

    public Claw(@NotNull Servo claw, @NotNull double openPos1, @NotNull double openPos2, @NotNull double closePos, @NotNull double restPos){
        this.claw = claw;
        this.posValues = new double[]{closePos, openPos1, openPos2, restPos};
    }
    public void ezSetPos(clawPos pos){
        claw.setPosition(posValues[pos.id]);
    }
    public void open(){
        ezSetPos(clawPos.OPEN1);
    }
    public void close(){
        ezSetPos(clawPos.CLOSE);
    }
    public void setPos(double pos){
        claw.setPosition(pos);
    }
    public double getPos(){
        return claw.getPosition();
    }
}
