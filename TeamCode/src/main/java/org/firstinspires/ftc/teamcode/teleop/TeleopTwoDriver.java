package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;



import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@TeleOp
public class TeleopTwoDriver extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx armMotor, slideMotor, fl, fr, bl, br, hangL, hangR = null;
    Servo rotation, wrist, clawL, clawR, hang;

    public double wristPar = 0, wristPerp = 0.55, wristOuttake = 0.75;
    public double clawLOpen = 1.0, clawLClose = 0.55, clawROpen = 0.0, clawRClose = 0.45;
    public double rotationPos = 0.5;
    public double armPar = 150, armUp = 1900;
    public int slideInterval = 15;
    public double hangClosed = 0.3, hangOpen = 1;

    // ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.2, armI = 0, armD = 0.001, armF = 0;
    //    extended PID
    double armPE = 0.2, armIE = 0, armDE = 0.001, armFE = 0;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.093, slideI = 0, slideD = 0.003, slideF = 0;
    double slidePE = 0.093, slideIE = 0, slideDE = 0.003, slideFE = 0;
    double slideTarget = 0.0;


    boolean rightBumperPrevState = false;
    boolean driver1yPrevState = false;
    boolean cameraOn = false;
    boolean claw = false;
    boolean clawOpen = false;
    boolean init = true;
    boolean slideExtended = false;
    boolean retractSlide = false;
    boolean retracted = true;
    boolean xHold = false;
    boolean xPress = false;
    boolean slideOuttake = false;



    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double driver1Multiplier = 1;
    double driver2Multiplier = 0.25;
    double armTempTarget = armPar;
    double slideMax = 2900;
    public enum Mode {
        REST,
        OUTTAKING,
        INTAKING,
        HANG
    }
    Mode mode = Mode.REST;


    public void initHardware() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        fl = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class,"backRightMotor");
        hangL = hardwareMap.get(DcMotorEx.class,"hangL");
        hangR = hardwareMap.get(DcMotorEx.class,"hangR");

        rotation = hardwareMap.get(Servo.class,"rotation");
        wrist = hardwareMap.get(Servo.class,"wrist");
        clawL = hardwareMap.get(Servo.class,"clawL");
        clawR = hardwareMap.get(Servo.class,"clawR");
        hang = hardwareMap.get(Servo.class,"hang");

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        hangL.setDirection(DcMotorEx.Direction.FORWARD);
        hangR.setDirection(DcMotorEx.Direction.REVERSE);
        hangL.setPower(0);
        hangR.setPower(0);
        hangL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist.setPosition(wristPerp);
        clawL.setPosition(clawLClose);
        clawR.setPosition(clawRClose);
        rotation.setPosition(0.5);
        hang.setPosition(hangClosed);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setPower(0);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();



        while (!isStopRequested()) {
            dashboardTelemetry.addData("status","running");

//  DRIVE
            double[] y = {-gamepad1.left_stick_y, -gamepad2.left_stick_y};
            double[] x = {gamepad1.left_stick_x * 1.1, gamepad2.left_stick_x * 1.1};
            double[] rx = {gamepad1.right_stick_x, gamepad2.right_stick_x};

            double[] denom = {
                    Math.max(Math.abs(y[0]) + Math.abs(x[0]) + Math.abs(rx[0]), 1),
                    Math.max(Math.abs(y[1]) + Math.abs(x[1]) + Math.abs(rx[1]), 1)
            };

            boolean driver1Active = gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0;

            int driverIndex = driver1Active ? 0 : 1; // Choose index based on which driver is active

            double driverMultiplier = driver1Active ? driver1Multiplier : driver2Multiplier;
            frontLeftPower = (y[driverIndex] + x[driverIndex] + rx[driverIndex]) / denom[driverIndex] * driverMultiplier;
            backLeftPower = (y[driverIndex] - x[driverIndex] + rx[driverIndex]) / denom[driverIndex] * driverMultiplier;
            frontRightPower = (y[driverIndex] - x[driverIndex] - rx[driverIndex]) / denom[driverIndex] * driverMultiplier;
            backRightPower = (y[driverIndex] + x[driverIndex] - rx[driverIndex]) / denom[driverIndex] * driverMultiplier;

            fl.setPower(frontLeftPower);
            fr.setPower(frontRightPower);
            bl.setPower(backLeftPower);
            br.setPower(backRightPower);

//  ARM & SLIDE PID


//            armMotor.setPower((armTarget > 0 && armTarget < 700) ? armPIDF(armTarget, armMotor) : 0);
//            slideMotor.setPower((slideTarget > 60 && slideTarget < 800) ? slidePIDF(slideTarget, slideMotor) : 0);

            if (armTarget >= 0 && armTarget <= 2200) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            }else{
                armMotor.setPower(0);
            }
            if (slideTarget >= 200 && slideTarget <= slideMax) {
                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
            }else{
                slideMotor.setPower(0);
            }


            if (mode==Mode.INTAKING){
                slideMax = 2900;
            }else{
                slideMax = 5270;
            }


//  CLAW
            if (gamepad2.a) {
                if (!claw) {
                    claw=true;
                    clawOpen = !clawOpen;
                }
            } else {
                claw = false;
            }

            if (!clawOpen){
                clawL.setPosition(clawLClose);
                clawR.setPosition(clawRClose);
            }else{
                clawL.setPosition(clawLOpen);
                clawR.setPosition(clawROpen);
            }

            if (gamepad2.x){
                if (!xHold){
                    xHold = true;
                    xPress = !xPress;
                }

            }else{
                xHold = false;
            }

            if (xPress){
                if (mode == Mode.INTAKING) {
                    armTempTarget = 400;
                    wrist.setPosition(wristPerp);
                    armTarget = armTempTarget;
                    retractSlide = true;

                }else if (mode == Mode.REST){
                    slideTarget = 200;
                }
            }
            xPress = false;

//  SLIDES
        slideTarget += (gamepad2.dpad_up && slideTarget<slideMax) ? slideInterval : 0;
        slideTarget -= (gamepad2.dpad_down && slideTarget>500) ? slideInterval : 0;
        slideTarget = Math.min(5270, Math.max(200, slideTarget));

        slideExtended = slideTarget > 300;

//  ARM

        armTempTarget += (gamepad1.left_trigger > 0) ? 3 : 0;
        armTempTarget -= (gamepad1.right_trigger > 0) ? 3 : 0;
        armTempTarget = Math.min(2300, Math.max(0, armTempTarget));

        armPar = (slideTarget > 300) ? 450 : 500;


//  MODES
        boolean rightBumperCurrentState = gamepad1.right_bumper;
        if (rightBumperCurrentState && !rightBumperPrevState) {
            if (mode == Mode.REST) {
                mode = Mode.OUTTAKING;
                slideInterval = 24;
                init = true;
            } else if (mode == Mode.OUTTAKING) {
                retractSlide=true;
                slideInterval = 18;
            }
        }
        rightBumperPrevState = rightBumperCurrentState;

        if (retractSlide) {
            slideTarget = 200;
            if (slideMotor.getCurrentPosition() < 500) {
                retractSlide = false;
                mode = Mode.REST;
                init = true;
            }
        }


        telemetry.addData("retract",retractSlide);


            boolean driver1yCurrentState = gamepad1.y;
            if (driver1yCurrentState && !driver1yPrevState) {
                if (mode == Mode.REST) {
                    mode = Mode.HANG;
                } else if (mode == Mode.HANG) {
                    mode = Mode.REST;
                }
                init = true;
            }
            driver1yPrevState = driver1yCurrentState;


            telemetry.addData("mode type",mode);
            switch (mode) {
/** REST */
                case REST:
                    if (init) {
                        wrist.setPosition(wristPerp);
                        slideTarget = 200;
                        armTempTarget = armPar;
                        rotation.setPosition(0.5);
                        hang.setPosition(hangClosed);
                    }
                    init = false;

// ARM POSITION
                    armTarget = armTempTarget;

// WRIST POSITION


// CHANGE TO INTAKING


                    if (slideTarget > 300) {
                        retracted = false;
                        mode = Mode.INTAKING;
                        init = true;
                    }


                    break;

/** INTAKING */
                case INTAKING:
                    if (init) {
                        wrist.setPosition(wristPar);
                        clawOpen = true;
                        cameraOn = false;
                        armTempTarget = armPar;
                    }
                    init = false;



//  ROTATION


                    if (gamepad2.left_trigger > 0 && rotationPos < 1) {
                        rotationPos += gamepad2.left_trigger / 80;
                        if (rotationPos > 1) rotationPos = 1; // Ensure upper bound
                    }
                    if (gamepad2.right_trigger > 0 && rotationPos > 0) {
                        rotationPos -= gamepad2.right_trigger / 80;
                        if (rotationPos < 0) rotationPos = 0; // Ensure lower bound
                    }
                    rotation.setPosition(rotationPos);


//  LOWER ARM
                    armTarget = (gamepad2.left_bumper) ? 325 : armTempTarget;

//  CHANGE TO REST
                    if (slideTarget <= 250){
                        mode = Mode.REST; //retract slide < 70; rest <= 80; intaking > 80
                    }

                    break;

/** OUTTAKING */
                case OUTTAKING:
                    if (init) {
                        armTempTarget = armUp;
                        slideOuttake = true;
                        rotation.setPosition(0.5);
                        wrist.setPosition(wristPar);


                    }
                    init = false;

                    if (slideOuttake && armTempTarget-armMotor.getCurrentPosition()<50){
                        slideTarget = slideMax;
                        slideOuttake = false;
                    }

                    if (gamepad2.left_bumper){
                        wrist.setPosition(wristOuttake);
                    }else{
                        wrist.setPosition(wristPar);
                    }

//  ARM
                    armTarget = armTempTarget;

                    break;

/** HANG */
                case HANG:
                    if (init) {
                        clawOpen = false;
                        armTarget = 2200;
                        slideTarget = 800;
                        wrist.setPosition(wristPar);
                        hang.setPosition(hangOpen);
                        rotation.setPosition(0.5);
                    }


                    break;
            }


            telemetry.addData("arm current",armMotor.getCurrentPosition());
            telemetry.addData("arm target",armTarget);
            telemetry.addData("slide current",slideMotor.getCurrentPosition());
            telemetry.addData("slide target",slideTarget);

            dashboardTelemetry.addData("arm current",armMotor.getCurrentPosition());
            dashboardTelemetry.addData("arm target",armTarget);
            dashboardTelemetry.addData("slide current",slideMotor.getCurrentPosition());
            dashboardTelemetry.addData("slide target",slideTarget);

            telemetry.addData("camera",cameraOn);
            telemetry.addData("rotation",rotationPos);
            telemetry.addData("init",init);

            telemetry.update();
            dashboardTelemetry.update();

        }
    }


    public double armPIDF(double target, DcMotorEx motor){
        if (slideExtended) {
            armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        } else {
            armPIDF.setPIDF(armP,armI,armD,armF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        telemetry.update();
        return output/5;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        if (mode == Mode.OUTTAKING){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        telemetry.update();
        return output/8;
    }

}
