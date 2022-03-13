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

@Autonomous(name = "Auto Red Outside")
public class Auto_Red_Outside extends LinearOpMode {

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
            case 0: // turn camera down
                timer = getRuntime();
                grab.setPosition(0.1);
                pivotPos = -40;
                state++;
                break;
            case 1: // wait for object detection
                if (getRuntime() >= timer + 3) {
                    previous = state;
                    state++;
                }
                break;
            case 2: // object detection
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
                                if (pos >= 165 && pos <= 466) {
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
            case 3: // turn left
                drivetrain.runMotorDistance(0.5, 0,-1000,1000,0);
                previous = state;
                state = -2;
                break;
            case 4: // turn ?
                drivetrain.runMotorDistance(0.5, 800,800,800,800);
                previous = state;
                state = -2;
                break;
            case 5:
                drivetrain.runMotorDistance(0.4, 0,-1030,1030,0);
                spinner.setPower(0.5);
                timer = getRuntime();
                previous = state;
                state = -2;
                break;
            case 6: // spinner runs for 5 seconds
                if (getRuntime() >= timer + 5) {
                    spinner.setPower(0);
                    previous = state;
                    state = -2;
                }
                break;
            case 7: // move right and extend slide
                drivetrain.runMotorDistance(0.8, 0,2400,-2400,0);
                slide.extend(-0.9, -4200);
                previous = state;
                state = -2;
                break;
            case 8:
                if (slide.stopSlide()) {
                    state++;
                }
                break;
//-----------------------------------Choose Pivot Position & Turn--------------------------------------
            case 9:
                if (position == 1) {
                    pivotPos = 350;
                }
                else if (position == 2) {
                    pivotPos = 75;
                    // TODO: Change state
                }
                else {
                    pivotPos = -73;
                }
                drivetrain.runMotorDistance(0.5, -1210,-1210,-1210,-1210);
                previous = state;
                state = -2;
                break;
//-----------------------------------Move forward--------------------------------------
            case 10:
                if (position == 1) {
                    drivetrain.runMotorDistance(0.50,-140,-140,140,140);
                    previous = state;
                    state = -2;
                    break;
                }
                else if (position == 2) {
                    drivetrain.runMotorDistance(0.5, -110,-110,110,110);
                    previous = state;
                    state = -2;
                    break;
                }
                else {
                    drivetrain.runMotorDistance(0.5, -50,-50,50,50);
                    previous = state;
                    state = -2;
                    break;
                }
//-----------------------------------Pre-load box release----------------------------------
            case 11:
                grab.setPosition(1);
                timer = getRuntime();
                state++;
                break;
            case 12:
                if (getRuntime() >= timer + 0.5) {
                    state++;
                }
                break;
            case 13:
                grab.setPosition(0.42);
                timer = getRuntime();
                state++;
                break;
            case 14:
                if (getRuntime() >= timer + 1) {
                    state++;
                }
                break;
//-----------------------------------Back & Towards Outside---------------------------------------
            case 15: // walk back
                drivetrain.runMotorDistance(0.50,150,150,-150,-150);
                previous = state;
                state = -2;
                break;
            case 16: // turn right
                drivetrain.runMotorDistance(0.50,-600,-600,-600,-600);
                previous = state;
                state = -2;
                break;
            case 17: // move into storage
                drivetrain.runMotorDistance(1,-3500,-3500,3500,3500);
                previous = state;
                state = -2;
                break;
            case 18: // pull slide back
                slide.extend(-0.9, 0);
                previous = state;
                state = -2;
            case 19:
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
