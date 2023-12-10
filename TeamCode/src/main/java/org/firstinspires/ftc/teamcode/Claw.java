package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.clawPos;
import org.jetbrains.annotations.NotNull;

public class Claw {
    private Servo claw;

    private double[] posValues;

    public Claw(@NotNull Servo claw, @NotNull double openPos1, @NotNull double openPos2, @NotNull double closePos, @NotNull double restPos){
        /**
         * @param claw claw servo
         * @param openPos1 partially opened servo position(drop one pixel)
         * @param openPos2 fully opened servo position(drop all pixels)
         * @param closePos fully closed servo position(grab pixels)
         * @param restPos servo position while claw in basket and awaits pixels from intake
         */
        this.claw = claw;
        this.posValues = new double[]{closePos, openPos1, openPos2, restPos};
    }
    public void ezSetPos(clawPos pos){
        claw.setPosition(posValues[pos.id]);
    }
//    public void ezSetPos(clawPoss pos){
//        claw.setPosition(pos);
//    }
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
