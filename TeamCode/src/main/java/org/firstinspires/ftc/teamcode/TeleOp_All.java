package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOp Freight Frenzy")
public class TeleOp_All extends LinearOpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private DcMotor pivot;
    private DcMotor slide;
    private DcMotor spinner;
    private Servo grab;
    private TouchSensor touch;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {
        m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        m4 = hardwareMap.dcMotor.get("M4"); // bottom rsight wheel
        pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        slide = hardwareMap.dcMotor.get("slide"); // linear slide
        spinner = hardwareMap.dcMotor.get("spinner"); // spinner
        grab = hardwareMap.servo.get("grab"); // grab
        touch = hardwareMap.touchSensor.get("touch");

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        int grabValue = -140;
        int balanceValue = 0;
        int topValue = 1000;
        int highValue = 1200;
        double pivotCorrection = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /* Mecanum Wheels */
                m1.setPower(-gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m2.setPower(gamepad2.left_stick_x + gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m3.setPower(-gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);
                m4.setPower(gamepad2.left_stick_x - gamepad2.left_stick_y + gamepad2.left_trigger - gamepad2.right_trigger);

                //-145
                //1300
                dashboardTelemetry.addData("Gamepad1.left_stick_y", gamepad1.left_stick_y);
                dashboardTelemetry.addData("Gamepad1.right_stick_y", gamepad1.right_stick_y);
                spinner.setPower(gamepad1.right_trigger);

                if (gamepad1.right_bumper) {
                    grab.setPosition(1);
                }
                else if (gamepad1.left_bumper) {
                    grab.setPosition(0.1);
                }
                else {
                    grab.setPosition(0.42);
                }

                if (slide.getCurrentPosition() < -4200 && gamepad1.left_stick_y < 0) {
                    slide.setPower(0);
                }
                else if (gamepad1.left_stick_y > 0 && touch.isPressed()) {
                    slide.setPower(0);
                }
                else {
                    slide.setPower(gamepad1.left_stick_y);
                }

                if (gamepad1.b) {
                    pivotStay(0.3, grabValue);
                }
                else if (gamepad1.x) {
                    pivotStay(0.3, highValue);
                }
                else if (gamepad1.y) {
                    pivotStay(0.3, topValue);
                }
                else {
                    pivotStay(0.3, balanceValue);
                }

                if (gamepad1.right_stick_y != 0) {
                    pivotCorrection = -gamepad1.right_stick_y;

                    grabValue += pivotCorrection;
                    highValue += pivotCorrection;
                    topValue += pivotCorrection;
                    balanceValue += pivotCorrection;
                }

                if (gamepad2.right_stick_y != 0) {
                    pivotCorrection = -gamepad1.right_stick_y;

                    grabValue += pivotCorrection;
                    highValue += pivotCorrection;
                    topValue += pivotCorrection;
                    balanceValue += pivotCorrection;
                }

                if (touch.isPressed()) {
                    dashboardTelemetry.addData("Touch", "Pressed");
                }

                dashboardTelemetry.addData("M1 Encoder", m1.getCurrentPosition());
                dashboardTelemetry.addData("M2 Encoder", m2.getCurrentPosition());
                dashboardTelemetry.addData("M3 Encoder", m3.getCurrentPosition());
                dashboardTelemetry.addData("M4 Encoder", m4.getCurrentPosition());
                dashboardTelemetry.addData("Pivot Correction", pivotCorrection);
                dashboardTelemetry.addData("Slides Encoder", slide.getCurrentPosition());
                dashboardTelemetry.addData("Pivot Position", pivot.getCurrentPosition());
                dashboardTelemetry.addData("Touch Sensor", touch.getValue());
                dashboardTelemetry.update();
            }
        }
    }

    public void pivotStay(double power, int p1){
        double pivotPower = power * ((p1 - pivot.getCurrentPosition()) / 100.0);
        if (pivotPower > 0.3) {
            pivotPower = 0.3;
        }
        else if (pivotPower < -0.3) {
            pivotPower = -0.3;
        }
        pivot.setPower(pivotPower);
    }
}
