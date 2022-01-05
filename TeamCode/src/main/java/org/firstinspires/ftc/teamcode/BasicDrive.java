package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "BasicDrive")
public class BasicDrive extends LinearOpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private DcMotor pivot;
    private DcMotor slide;
    private DcMotor spinner;
    private Servo grab;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {
        m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        m4 = hardwareMap.dcMotor.get("M4"); // bottom right wheel
        pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        slide = hardwareMap.dcMotor.get("slide"); // linear slide
        spinner = hardwareMap.dcMotor.get("spinner"); // spinner
        grab = hardwareMap.servo.get("grab"); // grab

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();
        if (opModeIsActive()) {
            m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            while (opModeIsActive()) {
                m1.setPower(-gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m2.setPower(gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m3.setPower(-gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m4.setPower(gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                pivot.setPower(gamepad1.left_stick_y * 0.4);
                spinner.setPower(gamepad1.right_trigger);

                if (gamepad1.right_bumper) {
                    grab.setPosition(0.75);
                }
                else if (gamepad1.left_bumper) {
                    grab.setPosition(0.1);
                }
                else {
                    grab.setPosition(0.5);
                }

                if (slide.getCurrentPosition() > 4700 && gamepad1.right_stick_y < 0) {
                    slide.setPower(0);
                }
                else if (slide.getCurrentPosition() < 0 && gamepad1.right_stick_y > 0) {
                    slide.setPower(0);
                }
                else {
                    slide.setPower(-gamepad1.right_stick_y);
                }

                //dashboardTelemetry.addData("Slides Encoder", slide.getCurrentPosition());
                dashboardTelemetry.addData("M1 Power", m1.getPower());
                dashboardTelemetry.addData("M2 Power", m2.getPower());
                dashboardTelemetry.addData("M3 Power", m3.getPower());
                dashboardTelemetry.addData("M4 Power", m4.getPower());
                //dashboardTelemetry.addData("Pivot Position", pivot.getCurrentPosition());
                dashboardTelemetry.update();

            }
        }
    }
}
