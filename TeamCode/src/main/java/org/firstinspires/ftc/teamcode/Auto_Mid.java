package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Auto Mid")
public class Auto_Mid extends LinearOpMode {

    private Drivetrain drivetrain;
    private DcMotor pivot;

    private int previous;
    private int state;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("M1"), // top left wheel
                hardwareMap.dcMotor.get("M2"), // bottom left wheel
                hardwareMap.dcMotor.get("M3"), // top right wheel
                hardwareMap.dcMotor.get("M4")  // bottom right wheel
        );

        pivot = hardwareMap.dcMotor.get("pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        previous = 0;
        state = 0;

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
                drivetrain.runMotorDistance(1, -2800,-2800,2800,2800);
                previous = state;
                state = -2;
                break;
            case 1:
                state = -1;
                break;
            case -2:
                if (drivetrain.stopMotor()) {
                    state = previous + 1;
                }
                break;
        }
        pivotStay(0.3, 0);
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
}
