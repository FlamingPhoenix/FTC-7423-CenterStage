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
    boolean liftGoDown = false;//change from gamepad
    public ArmAssemblyTeleOp(@NotNull Claw claw, @NotNull ServoArm servoArm, @NotNull Lift lift, Gamepad gamepad1, Gamepad gamepad2){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public void execute(){
        if(gamepad2.dpad_down){//LIFT DOWN, ARM RETRACT, CLAW CLOSE
            liftGoDown = true;
            claw.close();
            servoArm.retract();
        }
        if(gamepad2.dpad_up){//ARM EXTEND, CLAW CLOSE
            liftGoDown = false;
            claw.close();
            servoArm.extend();
        }


        if(gamepad2.a){//CLAW HALF OPEN
            claw.ezSetPos(clawPos.OPEN1);
        }
        if(gamepad1.b){//CLAW FULL OPEN
            claw.ezSetPos(clawPos.OPEN2);
        }
        if(gamepad2.x){//CLAW FULL CLOSED
            claw.close();
        }

        if(liftGoDown) {//WE WANT LIFT TO GO DOWN(IGNORE GAMEPAD)
            rightStickY = -0.111111;//CHANGE -- ~-0.5 // FOR TESTING, DON'T BREAK CLAW

            if(lift.getLiftPos()<5){//LIFT IS AT LOWEST
                liftGoDown = false;
                claw.ezSetPos(clawPos.REST);
            }
        }else{
            rightStickY = gamepad2.right_stick_y;//USE GAMEPAD INPUT
        }

        lift.setLiftDualMotor(rightStickY);
    }
}
