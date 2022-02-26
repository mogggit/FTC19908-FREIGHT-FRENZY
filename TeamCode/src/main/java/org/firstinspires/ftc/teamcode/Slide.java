package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Slide {

    private final DcMotor s1;
    private final TouchSensor touch;

    public Slide(DcMotor s1, TouchSensor touch) {
        this.s1 = s1;
        this.touch = touch;
    }

    public void extendSlide(double power, int p1) {
        if (touch.isPressed()) {
            s1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if ((touch.isPressed() && p1 < 0) || (!touch.isPressed())) {
            s1.setTargetPosition(p1);
            s1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            s1.setPower(power);
        }
    }

    public boolean stopSlide() {
        if (!s1.isBusy() || touch.isPressed()) {
            s1.setPower(0);
            s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
        else {
            return false;
        }
    }

}
