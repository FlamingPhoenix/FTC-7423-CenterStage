package org.firstinspires.ftc.teamcode;

public enum clawPos {
    CLOSE(0),//Close claw
    OPEN1(1),//Release bottom pixel
    OPEN2(2),//Release both pixels; open fully
    REST(3);//Intake mode; used when claw is in basket waiting for pixel
    public final int id;
    clawPos(final int _value){
        this.id = _value;
    }
}
