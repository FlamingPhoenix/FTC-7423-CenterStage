package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AUTO-RED-AUDIENCE", group="Linear Opmode")
public class AutoRedAudience extends LinearOpMode {
    DcMotor intake,fr,fl,br,bl;
    @Override
    public void runOpMode() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        //get prop position

        waitForStart();
        //go forward a bit
        //turn around
        //outtake to position
    }
}
