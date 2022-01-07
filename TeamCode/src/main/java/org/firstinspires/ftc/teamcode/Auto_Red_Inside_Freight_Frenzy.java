package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Red Inside Freight Frenzy")
public class Auto_Red_Inside_Freight_Frenzy extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};
    private static final String VUFORIA_KEY =
            "AW7MS5P/////AAABmbhCSjNBJkfFs+kp+0SOiHFqZSTkYVDULdxP11ncxw4EQzSyRq4EOiB4GBhwHNTrpMnzpmW6xnjHx4W9Z+wQrT7fMevji9eaAX/Zn+LQwm3VrXcZLz1qmqswkdRrEgea+8tLIfLGqlnPLTHyvFcQwI21X2nM9DPIOPgFX1H+mrJetXYSe5DcM6B1kkLMSP/Y4j6dtX4FADWxblGiTrryqV0D5r7B1OMTPMydiqbta46QVSm8CrDhP88TGZ6bvnPtlPli8PTev/CWl7qihRyh8U3I6J4CifMfNOF/fMfSIsho91WhZi3T6OB0ulsHtTxQrrVPte5SIBm7Vtstx05z4KcUnSZ4rybico/ME9juFkMO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private DcMotor pivot;
    private DcMotor slide;
    private DcMotor spinner;
    private Servo grab;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        m1 = hardwareMap.dcMotor.get("M1"); // top left wheel
        m2 = hardwareMap.dcMotor.get("M2"); // bottom left wheel
        m3 = hardwareMap.dcMotor.get("M3"); // top right wheel
        m4 = hardwareMap.dcMotor.get("M4"); // bottom right wheel
        pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        slide = hardwareMap.dcMotor.get("slide"); // linear slide
        spinner = hardwareMap.dcMotor.get("spinner"); // spinner
        grab = hardwareMap.servo.get("grab"); // grab

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboard.startCameraStream(tfod, 40);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.25, 16.0/9.0);
        }

        waitForStart();

        if (opModeIsActive()) {
            int position = -1;
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    dashboardTelemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        dashboardTelemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        dashboardTelemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        dashboardTelemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLeft() >= 455 && recognition.getRight() <= 756) {
                            position = 1; // middle
                        }
                        else if (recognition.getLeft() < 455) {
                            position = 0; // left
                        }
                        else {
                            position = 2; // right
                        }
                        dashboardTelemetry.addData(String.format("position (%d)", i), position);
                        i++;
                    }
                    dashboardTelemetry.update();
                }
            }

            double startTime = getRuntime();
            switch (position) {
                case 0:
                    // TODO: CASE 1
                    break;
                case 1:
                    grab.setPosition(0.5);

                    //Go to duck
                    runMotorDistance(0.2, -590, -500, 500, 590);
                    slideExtend(-1, -4200);
                    while (m1.isBusy() && m2.isBusy() && m3.isBusy() && m4.isBusy() && slide.isBusy()) {
                        dashboardTelemetry.addData("M1", m1.getCurrentPosition());
                        dashboardTelemetry.addData("M2", m2.getCurrentPosition());
                        dashboardTelemetry.addData("M3", m3.getCurrentPosition());
                        dashboardTelemetry.addData("M4", m4.getCurrentPosition());
                        dashboardTelemetry.update();
                        pivotStay(0.3, -10);
                    }
                    stopMotor();
                    stopSlide();

                    //Grab duck
                    startTime = getRuntime();
                    while (getRuntime() < startTime + 1.5) {
                        dashboardTelemetry.addData("Pivot", pivot.getCurrentPosition());
                        dashboardTelemetry.update();
                        pivotStay(0.3, -80);
                    }
                    grab.setPosition(0.78);
                    sleep(1000);

                    //Turn to hub

                    //Release duck

                    break;
                case 2:
                    // TODO: CASE 2
                    break;
                default:
                    break;
            }

            while (true) {
                pivotStay(0.3, 0);
                dashboardTelemetry.addData("Auto", "Ended");
                dashboardTelemetry.update();
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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
    }

    public void stopMotor() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideExtend(double power, int p1) {
        slide.setMode(DcMotor.RunMode.RESET_ENCODERS);
        slide.setTargetPosition(p1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }

    public void stopSlide() {
        pivot.setPower(0);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pivotStay(double power, int p1){
        double pivotPower = power * ((p1 - pivot.getCurrentPosition()) / 100.0);
        if (pivotPower > 0.5) {
            pivotPower = 0.5;
        }
        else if (pivotPower < -0.5) {
            pivotPower = -0.5;
        }
        pivot.setPower(pivotPower);
    }

    private void accelerateForward(double target1, double target2, double target3, double target4) {
        /* note: actual speed is set to target / 100 (for example, target = 30 --> moving at 0.3 power) */
        double cur_power1 = 0, cur_power2 = 0, cur_power3 = 0, cur_power4 = 0;
        while (true) {
            cur_power1 += 1; cur_power2 += 1; cur_power3 += 1; cur_power4 += 1;
            if (((cur_power1 >= target1) && (cur_power2 >= target2))
                    && ((cur_power3 >= target3) && (cur_power4 >= target4))) {
                break;
            }
            if (cur_power1 >= target1) { cur_power1 = target1; }
            if (cur_power2 >= target2) { cur_power2 = target2; }
            if (cur_power3 >= target3) { cur_power3 = target3; }
            if (cur_power4 >= target4) { cur_power4 = target4; }
            m1.setPower(cur_power1);
            m2.setPower(cur_power2);
            m3.setPower(-cur_power3);
            m4.setPower(-cur_power4);
            try {
                TimeUnit.MILLISECONDS.sleep(750); /* pause for 0.75 seconds between loops */
            }
            catch(InterruptedException ex) {
                Thread.currentThread().interrupt(); /* process got interrupted somehow */
            }
        }
    }
}
