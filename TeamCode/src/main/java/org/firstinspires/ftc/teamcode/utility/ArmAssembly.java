package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

public class armAssembly { //12-7: still in progress
    Claw claw;
    ServoArm servoArm;
    Lift lift;
    boolean clawOpened = false;
    boolean armExtended = false;
    int currentClawPos = 0;
    public armAssembly(@NotNull Claw claw, @NotNull ServoArm servoArm, @NotNull Lift lift){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
    }
    public void setLiftPos(double pos){
        lift.setLiftDualMotorPos(pos);
    }
    public void setCurrentClawPos(double pos){
        claw.setPos(pos);
    }
    public void setArmPos(double pos){
        servoArm.setPosition(pos);
    }
    public void setClaw(clawPos posID){
        claw.ezSetPos(posID);
        currentClawPos = posID.id;
        clawOpened = currentClawPos > 0;
    }
    public void setArm(boolean extended){
        if(clawOpened){
            claw.close();
            currentClawPos = 0;
            clawOpened = currentClawPos > 0;
        }
        if(extended){
            servoArm.extend();
        }else{
            servoArm.retract();
        }
        armExtended = extended;
    }
}
