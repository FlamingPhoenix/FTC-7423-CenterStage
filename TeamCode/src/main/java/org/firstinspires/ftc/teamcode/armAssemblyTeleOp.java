package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

public class armAssemblyTeleOp {
    Claw claw;
    servoArm servoArm;
    Lift lift;
    Gamepad gamepad1, gamepad2;
    boolean clawOpened = false;
    boolean armExtended = false;
    double rightStickY;
    boolean liftGoDown = false;//change from gamepad
    public armAssemblyTeleOp(@NotNull Claw claw, @NotNull servoArm servoArm, @NotNull Lift lift, Gamepad gamepad1, Gamepad gamepad2){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public void execute(){
        if(gamepad2.dpad_down){
            liftGoDown = true;
            claw.close();
            servoArm.retract();
        }
        if(gamepad2.dpad_up){
            liftGoDown = false;
            claw.close();
            servoArm.extend();
        }

        if(gamepad2.a){
            claw.ezSetPos(clawPos.OPEN1);
        }
        if(gamepad1.b){
            claw.ezSetPos(clawPos.OPEN2);
        }

        rightStickY = gamepad2.right_stick_y;

        if(liftGoDown) {
            rightStickY = -0.111111;

            if(lift.getLiftPos()<10){
                liftGoDown = false;
                claw.ezSetPos(clawPos.REST);
            }
        }

        lift.setLiftDualMotor(rightStickY);
    }
}
