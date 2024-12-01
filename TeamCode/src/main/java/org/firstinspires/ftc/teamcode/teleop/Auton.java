package org.firstinspires.ftc.teamcode.teleop;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@Autonomous
public class Auton extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx armMotor, slideMotor, fl, fr, bl, br, hangL, hangR = null;
    Servo rotation, wrist, clawR, hang;
    DcMotor par,perp;
    IMU imu;
    ElapsedTime delay = new ElapsedTime();

    public double wristPar = 0, wristPerp = 0.55, wristOuttake = 0.75;
    public double clawROpen = 0.0, clawRClose = 0.45;
    public double rotationPos = 0.5;
    public double armPar = 325, armUp = 1800;
    public int slideInterval = 15;
    public double hangClosed = 0.3, hangOpen = 1;

    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.0025, armI = 0, armD = 0.000023, armF = 0;
    //    extended PID
    double armPE = 0.003, armIE = 0, armDE = 0.000023, armFE = 0;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    double slidePE = 0.045, slideIE = 0, slideDE = 0.0004, slideFE = 0;
    double slideTarget = 0.0;
    double inPerTick =  120 /206845;


    boolean finished1,finished2,finished3,finished4,finished5,finished6,finished7,finished8,finished9,finished10 = false;
    boolean init = true;
    boolean slideOuttake = false;
    double start,curr;


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
        clawR = hardwareMap.get(Servo.class,"clawR");
        hang = hardwareMap.get(Servo.class,"hang");

//        par = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
//        perp = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
//        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontLeftMotor")));
//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backLeftMotor")));


//        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        clawR.setPosition(clawRClose);
        rotation.setPosition(0.5);

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

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armTarget = 800;
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(armMotor.getCurrentPosition() - armTarget) > 10 && timer.seconds() < 3) { // Safety timeout of 5 seconds
            double power = armPIDF(armTarget, armMotor); // Use the PID controller
            armMotor.setPower(power);

            // Optionally update telemetry
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armTarget);
            telemetry.update();
        }
        armMotor.setPower(0);
    }



    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();



        while (!isStopRequested()) {
//            dashboardTelemetry.addData("status","running");
////  DRIVE
//            if (armTarget >= 0) {
//                armMotor.setPower(armPIDF(armTarget, armMotor));
//            }else{
//                armMotor.setPower(0);
//            }
//            if (slideTarget >= 200) {
//                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
//            }else{
//                slideMotor.setPower(0);
//            }
//            init = true;
//            if (!finished1){
//                toHighRung();
//            }else{
//                if (!finished2) {
//                    armToRung();
//                }
//                else{
//                    if (!finished9){
//                        slideUp();
//                    }
//                }
//                } else{
//                    if (!finished9){
//                        armDown();
//                    }
//                }
//                else{
//                    if (!finished4){
//                        forward();
//                    }else{
//                        if (!finished5){
//                            strafeLeft();
//                        }else{
//                            if (!finished6){
//                                backToAscent();
//                            }else{
//                                if (!finished7){
//                                    rotate();
//                                }else{
//                                    if (!finished8){
//                                        park();
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }



// PID
                if (armTarget >= 0) {
                    armMotor.setPower(armPIDF(armTarget, armMotor));
                } else {
                    armMotor.setPower(0);
                }

                if (slideTarget >= 200) {
                    slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
                } else {
                    slideMotor.setPower(0);
                }



// AUTON ACTIONS
                if (!finished1) {
                    toHighRung();
                } else if (!finished2) {
                    armToRung();
                } else if (!finished9) {
                    slideUp();
                }else if(!finished3){
                    armDown();
                }

                telemetry.addData("slide",slideTarget);
                telemetry.addData("slidecurr",slideMotor.getCurrentPosition());
                telemetry.addData("arm",armTarget);
                telemetry.addData("armcurr",armMotor.getCurrentPosition());
                telemetry.addData("par",fl.getCurrentPosition());
                telemetry.addData("perp", bl.getCurrentPosition());
                telemetry.addData("finished1",finished1);
                telemetry.addData("finished2",finished2);
                telemetry.addData("finished3",finished3);


                telemetry.update();
                dashboardTelemetry.update();
        }
    }

    public void toHighRung(){
        if (fl.getCurrentPosition()<-30672){ //to high rung //29 inches //28672
            finished1 = true;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }else{
            fl.setPower(-1);
            fr.setPower(-1);
            bl.setPower(-1);
            br.setPower(-1);
        }
    }
    public void armToRung() {
        armTarget = armUp;
        slideOuttake = true;
        slideTarget = 700;
        if (slideTarget - slideMotor.getCurrentPosition()<100) {
            wrist.setPosition(wristOuttake);
            waitSec(2);
            finished2 = true;
        }

//        rotation.setPosition(0.5);
//        wrist.setPosition(wristPar);

//        if (slideOuttake && armTarget - armMotor.getCurrentPosition() < 1000 && slideTarget - slideMotor.getCurrentPosition()<500) {
//            slideOuttake = false;

//        }

    }
    public void slideUp(){
        slideTarget = 3000;
        waitSec(2);
        if (slideTarget - slideMotor.getCurrentPosition() < 50) {
            clawR.setPosition(clawROpen);
            waitSec(2);
            finished9 = true;


        }

    }

    public void armDown(){
        slideTarget = 200;
        wrist.setPosition(wristPerp);

        armTarget = armPar;
        rotation.setPosition(0.5);
        if (armMotor.getCurrentPosition()<500){
            finished3 = true;
        }
    }
    public void forward(){
        if (fl.getCurrentPosition()>-28694) { //5 inches
            finished4 = true;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }else {
            fl.setPower(1);
            fr.setPower(1);
            bl.setPower(1);
            br.setPower(1);
        }

    }
    public void strafeLeft(){
        if (bl.getCurrentPosition()>28694){
            finished5 = true;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

        }else{
            fl.setPower(1);
            fr.setPower(-1);
            bl.setPower(-1);
            br.setPower(1);
        }
    }
    public void backToAscent(){
        if (fl.getCurrentPosition()<-43041){
            finished6 = true;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }else{
            fl.setPower(-1);
            fr.setPower(-1);
            bl.setPower(-1);
            br.setPower(-1);
        }
    }

    public void rotate(){

    }

    public void park(){
        finished8 = true;
        armTarget = 1200;
        slideTarget = 1300;
        wrist.setPosition(wristPar);
    }

    public double armPIDF(double target, DcMotorEx motor){
        if (slideTarget>500) {
            armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        } else {
            armPIDF.setPIDF(armP,armI,armD,armF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        telemetry.update();
        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        if (armTarget>500){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        telemetry.update();
        return output;
    }

    public void waitSec(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            // Update the arm and slide motors with their PID control
            if (armTarget >= 0) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            } else {
                armMotor.setPower(0);
            }
            if (slideTarget >= 200) {
                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
            } else {
                slideMotor.setPower(0);
            }

            // Optionally, update telemetry during the wait
            telemetry.addData("Waiting", "%.1f seconds remaining", seconds - timer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }






}


