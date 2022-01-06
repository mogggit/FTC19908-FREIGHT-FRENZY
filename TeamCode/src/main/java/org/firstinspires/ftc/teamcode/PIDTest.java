package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PIDTest")
public class PIDTest extends LinearOpMode {
    private DcMotor pivot;
    PIDController pidPivot;
    double correction;

    @Override
    public void runOpMode() {
        pivot = hardwareMap.dcMotor.get("pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidPivot = new PIDController(1, 1, 0);
        waitForStart();
        pidPivot.setSetpoint(-145);
        pidPivot.setOutputRange(-0.5, 0.5);
        pidPivot.setInputRange(-150, 1500);
        pidPivot.enable();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                correction = pidPivot.performPID(pivot.getCurrentPosition());
                pivot.setPower(0.1 + correction);
            }
        }
        pivot.setPower(0);
    }
}
