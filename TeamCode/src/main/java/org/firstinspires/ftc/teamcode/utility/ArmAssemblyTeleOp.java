package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.clawPos;
import org.jetbrains.annotations.NotNull;

public class ArmAssemblyTeleOp {
    Claw claw;
    ServoArm servoArm;
    Lift lift;
    Gamepad gamepad1, gamepad2;
    boolean clawOpened = false;
    boolean armExtended = false;
    double rightStickY;
    boolean returnToRest = false;//change from gamepad
    boolean extendToDrop = false;
    public ArmAssemblyTeleOp(@NotNull Claw claw, @NotNull ServoArm servoArm, @NotNull Lift lift, Gamepad gamepad1, Gamepad gamepad2){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public void execute(){
        if(gamepad2.dpad_down){//LIFT DOWN, ARM RETRACT, CLAW CLOSE
            extendToDrop = false;
            returnToRest = true;
            claw.close();
            servoArm.retract();
        }
        if(gamepad2.dpad_up){//LIFT UP, ARM EXTEND, CLAW CLOSE
            extendToDrop = true;
            returnToRest = false;
            claw.close();
            servoArm.extend();
        }


        if(gamepad2.a){//CLAW HALF OPEN
            claw.ezSetPos(clawPos.OPEN1);
        }
        if(gamepad2.b){//CLAW FULL OPEN
            claw.ezSetPos(clawPos.OPEN2);
        }
        if(gamepad2.x){//CLAW FULL CLOSED
            claw.close();
        }

        rightStickY = gamepad2.right_stick_y;

        if(Math.abs(rightStickY) >= 0.01){ //If driver is giving an input, they probably want to override the lift power
            extendToDrop = false;
            returnToRest = false;
        }

        if(returnToRest) {
            rightStickY = -0.111111;//CHANGE --~-0.5 // FOR TESTING, DON'T BREAK CLAW

            if(lift.getLiftPos()<5){//LIFT IS AT LOWEST
                claw.ezSetPos(clawPos.REST);
                returnToRest = false;
            }
        }
        if(extendToDrop) {
            rightStickY = 0.25;//set to low speed for testing
        }

        lift.setLiftDualMotor(rightStickY);
    }
}
