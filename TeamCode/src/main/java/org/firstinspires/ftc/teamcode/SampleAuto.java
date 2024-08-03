package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name="SampleAuto")
public class SampleAuto extends LinearOpMode {
    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    public void runOpMode() {
        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SWAP THESE IF NEEDED
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
//        driveBR.setDirection(DcMotorEx.Direction.REVERSE);
        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
//        driveFR.setDirection(DcMotorEx.Direction.REVERSE);

        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = 100;
        driveBL.setTargetPosition(targetPosition);
        driveBR.setTargetPosition(targetPosition);
        driveFL.setTargetPosition(targetPosition);
        driveFR.setTargetPosition(targetPosition);

        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 0.7;
        driveBL.setPower(power);
        driveBR.setPower(power);
        driveFL.setPower(power);
        driveFR.setPower(power);

        while(driveBL.isBusy() || driveBR.isBusy() ||driveFL.isBusy() ||driveFR.isBusy()) {

        }

        driveBL.setPower(0.0);
        driveBR.setPower(0.0);
        driveFL.setPower(0.0);
        driveFR.setPower(0.0);

    }
}
