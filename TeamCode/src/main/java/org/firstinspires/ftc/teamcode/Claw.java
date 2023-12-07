package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class Claw {
    private Servo claw;

    private double openPos1, closePos,openPos2,restPos;

    double posIDS[] = {restPos,closePos,openPos1,openPos2};
    public Claw(@NotNull Servo claw,@NotNull double openPos,@NotNull double closePos){
        this.claw = claw;
        this.openPos1 = openPos;
        this.closePos = closePos;
    }
    public Claw(@NotNull Servo claw, @NotNull double openPos1, @NotNull double openPos2, @NotNull double closePos, @NotNull double restPos){
        this.claw = claw;
        this.openPos1 = openPos1;
        this.openPos2 = openPos2;
        this.closePos = closePos;
        this.restPos = restPos;
    }
    public void ezSetPos(int posID){
        if(posID<posIDS.length) {
            claw.setPosition(posIDS[posID]);
        }else{
            throw new RuntimeException("posID out of bounds: "+posID+" (max: "+posIDS.length+")\ntry using setPos()?");
        }
    }
    public void open(){
        claw.setPosition(openPos1);
    }
    public void close(){
        claw.setPosition(closePos);
    }
    public void setPos(double pos){
        claw.setPosition(pos);
    }
    public double getPos(){
        return claw.getPosition();
    }
}
