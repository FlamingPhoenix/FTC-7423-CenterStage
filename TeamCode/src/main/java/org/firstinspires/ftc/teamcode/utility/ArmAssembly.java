package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.clawPos;
import org.jetbrains.annotations.NotNull;

public class ArmAssembly { //12-7: still in progress
    Claw claw;
    ServoArm servoArm;
    Lift lift;
    public boolean clawOpened = false;
    public boolean armExtended = false;
    int currentClawPos = 0;
    public ArmAssembly(@NotNull Claw claw, @NotNull ServoArm servoArm, @NotNull Lift lift){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
    }
    public void retract(){
        servoArm.retract();
        armExtended = false;
        setLiftPos(0);
        closeClaw();
        clawOpened = false;
    }
    public void setLiftPos(double pos){
        lift.setLiftDualMotorPos(pos);
    }
    public void setCurrentClawPos(double pos){
        /**
         * */
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
    public void closeClaw(){
        claw.close();
        clawOpened = false;
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
