package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

public class armAssembly { //12-7: still in progress
    Claw claw;
    servoArm servoArm;
    Lift lift;
    boolean clawOpened = false;
    boolean armExtended = false;
    int clawPos = 0;
    ElapsedTime timer;
    //extend arm only when claw is closed
    //rest claw when arm is retracted
    public armAssembly(@NotNull Claw claw, @NotNull servoArm servoArm, @NotNull Lift lift){
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
    public void setClawPos(double pos){
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
    public void setClaw(int posID){
        claw.ezSetPos(posID);
        clawPos = posID;
        if(clawPos > 0){
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
