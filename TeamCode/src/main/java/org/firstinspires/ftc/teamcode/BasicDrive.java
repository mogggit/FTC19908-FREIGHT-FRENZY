package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BasicDrive")
public class BasicDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        DcMotor m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        DcMotor m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        DcMotor m4 = hardwareMap.dcMotor.get("M4"); // bottom right wheel
        DcMotor pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        DcMotor slide = hardwareMap.dcMotor.get("slide"); // linear slide
        DcMotor spinner = hardwareMap.dcMotor.get("spinner"); // spinner

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
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

                if (slide.getCurrentPosition() > 4700 && gamepad1.right_stick_y < 0) {
                    slide.setPower(0);
                }
                else if (slide.getCurrentPosition() < 0 && gamepad1.right_stick_y > 0) {
                    slide.setPower(0);
                }
                else {
                    slide.setPower(-gamepad1.right_stick_y);
                }

                telemetry.addData("Slides Encoder: ", slide.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
