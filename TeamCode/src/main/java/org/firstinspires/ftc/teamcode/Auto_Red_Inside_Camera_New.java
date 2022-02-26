package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Auto Red Camera New")
public class Auto_Red_Inside_Camera_New extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};
    private static final String VUFORIA_KEY =
            "AW7MS5P/////AAABmbhCSjNBJkfFs+kp+0SOiHFqZSTkYVDULdxP11ncxw4EQzSyRq4EOiB4GBhwHNTrpMnzpmW6xnjHx4W9Z+wQrT7fMevji9eaAX/Zn+LQwm3VrXcZLz1qmqswkdRrEgea+8tLIfLGqlnPLTHyvFcQwI21X2nM9DPIOPgFX1H+mrJetXYSe5DcM6B1kkLMSP/Y4j6dtX4FADWxblGiTrryqV0D5r7B1OMTPMydiqbta46QVSm8CrDhP88TGZ6bvnPtlPli8PTev/CWl7qihRyh8U3I6J4CifMfNOF/fMfSIsho91WhZi3T6OB0ulsHtTxQrrVPte5SIBm7Vtstx05z4KcUnSZ4rybico/ME9juFkMO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    private Drivetrain drivetrain;
    private DcMotor pivot;
    private DcMotor spinner;
    private Slide slide;
    private Servo grab;

    private int previous;
    private int state;
    private int position;
    private int pivotPos;
    private double timer;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.
            tfod.setZoom(1, 16.0/9.0);
        }

        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("M1"), // top left wheel
                hardwareMap.dcMotor.get("M2"), // bottom left wheel
                hardwareMap.dcMotor.get("M3"), // top right wheel
                hardwareMap.dcMotor.get("M4")  // bottom right wheel
        );

        slide = new Slide(
                hardwareMap.dcMotor.get("slide"),
                hardwareMap.touchSensor.get("touch")
        );

        pivot = hardwareMap.dcMotor.get("pivot"); // pivot
        spinner = hardwareMap.dcMotor.get("spinner"); // spinner
        grab = hardwareMap.servo.get("grab"); // grab
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboard.startCameraStream(tfod, 40);

        previous = 0;
        state = 0;
        pivotPos = 0;

        waitForStart();
        while (opModeIsActive()) {
            mainFSM();
            if (state == -1) {
                break;
            }
        }
    }

    /*
    Main Finite State Machine
    States:
    -1 -> end program
    -2 -> stop motor
    0~ -> moves
     */
    public void mainFSM() {
        switch (state) {
            case 0:
                timer = getRuntime();
                grab.setPosition(0.1);
                pivotPos = -50;
                state++;
                break;
            case 1:
                if (getRuntime() >= timer + 3) {
                    previous = state;
                    state++;
                }
                break;
            case 2:
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        dashboardTelemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            dashboardTelemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            dashboardTelemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            dashboardTelemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            double pos = (recognition.getLeft() + recognition.getRight()) / 2;

                            if (pos >= 222 && pos <= 430) {
                                position = 2; // middle
                            }
                            else if (recognition.getLeft() < 222) {
                                position = 1; // left
                            }
                            else {
                                position = 3; // right
                            }
                            dashboardTelemetry.addData(String.format("position (%d)", i), position);
                            i++;
                        }
                        dashboardTelemetry.update();
                        previous = state;
                        state++;
                    }
                }
                break;
            case 3:
                drivetrain.runMotorDistance(0.5, -450,-1200,1200,450);
                slide.extend(-0.5, -4200);

                if (position == 1) {
                    pivotPos = 320;
                }
                else if (position == 2) {
                    pivotPos = 37;
                }
                else {
                    pivotPos = -100;
                }

                previous = state;
                state = -2;
                break;
            case 4:
                drivetrain.runMotorDistance(0.5, 400,400,400,400);
                previous = state;
                state = -2;
                break;
            case 5:
                if (slide.stopSlide()) {
                    previous = state;
                    state++;
                }
                break;
            case 6:
                grab.setPosition(1);
                timer = getRuntime();
                previous = state;
                state++;
                break;
            case 7:
                if (getRuntime() >= timer + 0.5) {
                    previous = state;
                    state++;
                }
                break;
            case 8:
                grab.setPosition(0.42);
                previous = state;
                state++;
                break;
            case 9:
                state = -1;
                break;
            case -2:
                if (drivetrain.stopMotor()) {
                    state = previous + 1;
                }
                break;
        }
        pivotStay(0.35, pivotPos);
    }

    // maintain pivot position
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

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
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
}
