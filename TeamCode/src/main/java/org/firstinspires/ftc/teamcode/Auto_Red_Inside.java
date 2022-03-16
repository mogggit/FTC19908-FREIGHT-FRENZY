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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Autonomous(name = "Auto Red Inside")
public class Auto_Red_Inside extends LinearOpMode {

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
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                pivotPos = -40;
                state++;
                break;
            case 1:
                if (getRuntime() >= timer + 5) {
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

                            if (recognition.getLabel() == "Duck") {
                                double pos = (recognition.getLeft() + recognition.getRight()) / 2;
                                if (pos >= 165 && pos <= 434) {
                                    position = 2; // middle
                                }
                                else if (pos < 165) {
                                    position = 1; // left
                                }
                                else {
                                    position = 3; // right
                                }
                                dashboardTelemetry.addData(String.format("position (%d)", i), position);
                                dashboardTelemetry.addData(String.format("mid value (%d)", i), pos);
                            }
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
                    pivotPos = 350;
                    state = 4;
                }
                else if (position == 2) {
                    pivotPos = 85;
                    state = 16;
                }
                else {
                    state = 24;
                }
                previous = state - 1;
                state = -2;
                break;
//-----------------------------------Start of layer 1----------------------------------
            case 4:
                drivetrain.runMotorDistance(0.5, 400,400,400,400);
                previous = state;
                state = -2;
                break;
            case 5:
                if (slide.stopSlide()) {
                    state++;
                }
                break;
            case 6:
                drivetrain.runMotorDistance(0.5, -360,-360,360,360);
                previous = state;
                state = -2;
                break;
//-----------------------------------Per-load box release----------------------------------
            case 7:
                grab.setPosition(1);
                timer = getRuntime();
                state++;
                break;
            case 8:
                if (getRuntime() >= timer + 0.5) {
                    state++;
                }
                break;
            case 9:
                grab.setPosition(0.42);
                timer = getRuntime();
                state++;
                break;
            case 10:
                if (getRuntime() >= timer + 1) {
                    if (position == 1) {
                        state = 11;
                    }
                    else if (position == 2) {
                        state = 18;
                    }
                    else {
                        state = 25;
                        pivotPos = -30;
                    }
                }
                break;
//-----------------------------------------------------------------------------------------
            case 11:
                drivetrain.runMotorDistance(0.5, 300,300,-300,-300);
                previous = state;
                state = -2;
                break;
            case 12:
                drivetrain.runMotorDistance(0.5, -1440,-1440,-1440,-1440);
                previous = state;
                state = -2;
                break;
            case 13:
                pivotPos = 0;
                drivetrain.runMotorDistance(1, -2300,-2300,2300,2300);
                slide.extend(0.5, 0);
                pivotPos = 0;
                previous = state;
                state = -2;
                break;
            case 14:
                if (slide.stopSlide()) {
                    state++;
                }
                break;
            case 15:
                state = -1;
                break;
//-----------------------------------Start of layer 2----------------------------------
            case 16:
                drivetrain.runMotorDistance(0.51, 400,400,400,400);
                previous = state;
                state = -2;
                break;
            case 17:
                drivetrain.runMotorDistance(0.50, -100,-100,100,100);
                previous = state;
                state = -2;
                break;
            case 18:
                pivotPos = 80;
                state = 7;
                break;
            case 19:
                pivotPos = 100;
                drivetrain.runMotorDistance(0.50, 200,200,-200,-200);
                previous = state;
                state = -2;
                break;
            case 20:
                drivetrain.runMotorDistance(0.5, -1450,-1450,-1450,-1450);
                pivotPos = 0;
                previous = state;
                state = -2;
                break;
            case 21:
                pivotPos = 0;
                drivetrain.runMotorDistance(1, -2300,-2300,2300,2300);
                slide.extend(0.5, 0);
                previous = state;
                state = -2;
                break;
            case 22:
                if (slide.stopSlide()) {
                    state++;
                }
                break;
            case 23:
                state = -1;
                break;
//-----------------------------------Start of layer 3----------------------------------
            case 24:
                pivotPos = -70;
                drivetrain.runMotorDistance(0.50, 330,330,330,330);
                previous = state;
                state = -2;
                break;
            case 25:
                drivetrain.runMotorDistance(0.5, -0,-0,0,0);
                previous = state;
                state = -2;
                break;
            case 26:
                state = 7;
                break;
            case 27:
                pivotPos = -40;
                drivetrain.runMotorDistance(0.50, -1400,-1400,-1400,-1400);
                previous = state;
                state = -2;
                break;
            case 28:
                pivotPos = 0;
                drivetrain.runMotorDistance(1, -2300,-2300,2300,2300);
                slide.extend(0.5, 0);
                previous = state;
                state = -2;
                break;
            case 29:
                if (slide.stopSlide()) {
                    state++;
                }
                break;
            case 30:
                state = -1;
                break;
//-----------------------------------Stop motor ---------------------------------------
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
