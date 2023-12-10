package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.clawPos;
import org.jetbrains.annotations.NotNull;

public class ArmAssembly { //12-7: still in progress
    Claw claw;
    ServoArm servoArm;
    Lift lift;
    boolean clawOpened = false;
    boolean armExtended = false;
    int currentClawPos = 0;
    ElapsedTime timer;
    //extend arm only when claw is closed
    //rest claw when arm is retracted
    public ArmAssembly(@NotNull Claw claw, @NotNull ServoArm servoArm, @NotNull Lift lift){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
        timer = new ElapsedTime();
    }
    public void each(){

    }
    public void setLift(){
        lift.setLiftDualMotor();
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
    public void retractArm(){
        claw.close();
        servoArm.retract();
        armExtended = false;
        clawOpened = true;
    }
    public void extendArm(){
        servoArm.extend();
        armExtended = true;
    }
    public void setClaw(clawPos posID){
        claw.ezSetPos(posID);
        currentClawPos = posID.id;
        if(currentClawPos > 0){
            clawOpened = true;
        }else{
            clawOpened = false;
        }
    }
    public void closeClaw(){
        claw.close();
        clawOpened = false;
    }
    public void setArm(boolean extended){
        if(clawOpened){
            claw.close();
        }
        if(extended){
            servoArm.extend();
        }else{
            servoArm.retract();
        }
        armExtended = extended;
    }
}
