package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor_Encoder_test extends LinearOpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;

    @Override
    public void runOpMode() throws InterruptedException {

        m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        m4 = hardwareMap.dcMotor.get("M4"); // bottom right wheel

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("test", "test");
            }
        }
    }
}
