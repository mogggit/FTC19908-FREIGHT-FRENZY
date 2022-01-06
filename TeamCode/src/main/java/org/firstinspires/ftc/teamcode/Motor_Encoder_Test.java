package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Motor Encoder Test")
public class Motor_Encoder_Test extends LinearOpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        m4 = hardwareMap.dcMotor.get("M4"); // bottom right wheel

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                dashboardTelemetry.addData("Hello", "Hello");
                dashboardTelemetry.update();
                runMotorDistance(0.5, 1000, 0, 0, 0);
                sleep(1000000);
            }
        }
    }

    public void runMotorDistance(double power, int p1, int p2, int p3, int p4) {

        m1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        m2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        m3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        m4.setMode(DcMotor.RunMode.RESET_ENCODERS);

        m1.setTargetPosition(p1);
        m2.setTargetPosition(p2);
        m3.setTargetPosition(p3);
        m4.setTargetPosition(p4);

        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);

        while (m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy()) {}

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
