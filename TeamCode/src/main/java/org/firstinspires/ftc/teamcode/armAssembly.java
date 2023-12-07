package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class armAssembly { //12-3: still in progress
    Claw claw;
    servoArm servoArm;
    Lift lift;
    boolean clawExtended = false;
    boolean armExtended = false;
    public armAssembly(@NotNull Claw claw, @NotNull servoArm servoArm, @NotNull Lift lift){
        this.claw = claw;
        this.servoArm = servoArm;
        this.lift = lift;
    }

}
