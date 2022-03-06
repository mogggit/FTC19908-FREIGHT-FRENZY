package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOp All") // name of this program shown in ds
public class TeleOp_All extends LinearOpMode {

    private Drivetrain drivetrain;
    private DcMotor pivot;
    private DcMotor slide;
    private DcMotor spinner;
    private Servo grab;
    private TouchSensor touch;

    // combines dashboard telemetry and ds telemetry
    private MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() {
        
        // instantiate every part of the robot
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("M1"), // top left wheel
                hardwareMap.dcMotor.get("M2"), // bottom left wheel
                hardwareMap.dcMotor.get("M3"), // top right wheel
                hardwareMap.dcMotor.get("M4")  // bottom right wheel
        );
        pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        slide = hardwareMap.dcMotor.get("slide"); // linear slide
        spinner = hardwareMap.dcMotor.get("spinner"); // spinner
        grab = hardwareMap.servo.get("grab"); // grab servo
        touch = hardwareMap.touchSensor.get("touch"); // touch sensor for slide

        // set pivot, slide, spinner, and drivetrain to teleOp mode
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.driveMode();

        // encoder values for different pivot positionsd
        int grabValue = -140; // almost touch the ground to grab game elements
        int balanceValue = 0; // horinzontal
        int topValue = 1000; // verticle
        int highValue = 1200; // high level
        int lowValue = -90; // low level
        double pivotCorrection = 0; // manual correction

        // wait for start button to be pressed
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // mecanum Wheels
                drivetrain.drive(gamepad2);

                // spinner for duck
                spinner.setPower(gamepad1.right_trigger);
                
                // grab servo
                if (gamepad1.right_bumper) {
                    grab.setPosition(1);
                }
                else if (gamepad1.left_bumper) {
                    grab.setPosition(0.1);
                }
                else {
                    grab.setPosition(0.42);
                }

                // slide control with fail-safe using touch sensor
                if (slide.getCurrentPosition() < -4200 && gamepad1.left_stick_y < 0) {
                    slide.setPower(0);
                }
                else if (gamepad1.left_stick_y > 0 && touch.isPressed()) {
                    slide.setPower(0);
                }
                else {
                    slide.setPower(gamepad1.left_stick_y);
                }

                // pivot
                if (gamepad1.b) {
                    pivotStay(0.3, grabValue);
                }
                else if (gamepad1.x) {
                    pivotStay(0.3, highValue);
                }
                else if (gamepad1.y) {
                    pivotStay(0.3, topValue);
                }
                else if (gamepad1.a) {
                    pivotStay(0.3, lowValue);
                }
                else {
                    pivotStay(0.3, balanceValue);
                }

                // driver 1 manual correction for pivot
                if (gamepad1.right_stick_y != 0) {
                    pivotCorrection = -gamepad1.right_stick_y;
                    grabValue += pivotCorrection;
                    highValue += pivotCorrection;
                    topValue += pivotCorrection;
                    balanceValue += pivotCorrection;
                    lowValue += pivotCorrection;
                }

                // driver 2 manual correction for pivot
                if (gamepad2.right_stick_y != 0) {
                    pivotCorrection = -gamepad1.right_stick_y;
                    grabValue += pivotCorrection;
                    highValue += pivotCorrection;
                    topValue += pivotCorrection;
                    balanceValue += pivotCorrection;
                    lowValue += pivotCorrection;
                }

                // telemetry data to dashboard and ds
                tel.addData("Gamepad1.left_stick_y", gamepad1.left_stick_y);
                tel.addData("Gamepad1.right_stick_y", gamepad1.right_stick_y);
                tel.addData("M1 Encoder", drivetrain.getEncoder("m1"));
                tel.addData("M2 Encoder", drivetrain.getEncoder("m2"));
                tel.addData("M3 Encoder", drivetrain.getEncoder("m3"));
                tel.addData("M4 Encoder", drivetrain.getEncoder("m4"));
                tel.addData("Pivot Correction", pivotCorrection);
                tel.addData("Slides Encoder", slide.getCurrentPosition());
                tel.addData("Pivot Position", pivot.getCurrentPosition());
                tel.addData("Touch Sensor", touch.getValue());
                tel.update();
            }
        }
    }

    // prevent pivot from tipping
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
