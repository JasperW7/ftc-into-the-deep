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
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;



@Config
@TeleOp
public class TeleopOneDriverSpecimen extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx armMotor, slideMotor, fl, fr, bl, br, hangL, hangR = null;
    Servo rotation, wrist, clawR, hang;

    public double wristPar = 0, wristPerp = 0.55, wristOuttake = 0.75;
    public double clawROpen = 0.0, clawRClose = 0.45;
    public double rotationPos = 0.5;
    public double armPar = 325, armUp = 1700;
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

    boolean rightBumperPrevState = false;
    boolean hangPrev = false;
    boolean claw = false;
    boolean clawOpen = false;
    boolean init = true;
    boolean slideExtended = false;
    boolean retractSlide = false;
    boolean retracted = true;
    boolean slideOuttake = false;
    boolean micro = false;
    boolean intakePrev = false;



    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double armTempTarget = armPar;
    double slideMax = 3000;

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
        clawR = hardwareMap.get(Servo.class,"clawR");
        hang = hardwareMap.get(Servo.class,"hang");

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

//
//                        /^--^\     /^--^\     /^--^\
//                        \____/     \____/     \____/
//                       /      \   /      \   /      \
//                      |        | |        | |        |
//                       \__  __/   \__  __/   \__  __/
//  |^|^|^|^|^|^|^|^|^|^|^|^\ \^|^|^|^/ /^|^|^|^|^\ \^|^|^|^|^|^|^|^|^|^|^|^|
//  | | | | | | | | | | | | |\ \| | |/ /| | | | | | \ \ | | | | | | | | | | |
//  ########################/ /######\ \###########/ /#######################
//  | | | | | | | | | | | | \/| | | | \/| | | | | |\/ | | | | | | | | | | | |
//  |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|


    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();



        while (!isStopRequested()) {
            dashboardTelemetry.addData("status","running");
//  DRIVE

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (!micro) {

                double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx),1);
                frontLeftPower = (y + x + rx) / denom;
                backLeftPower = (y - x + rx) / denom;
                frontRightPower = (y - x - rx) / denom;
                backRightPower = (y + x - rx) / denom;

                fl.setPower(frontLeftPower);
                fr.setPower(frontRightPower);
                bl.setPower(backLeftPower);
                br.setPower(backRightPower);
            }else{
                //TODO trig calculation for rotation

                frontLeftPower = rx/3;
                backLeftPower = rx/3;
                frontRightPower = -rx/3;
                backRightPower = -rx/3;

                fl.setPower(frontLeftPower);
                fr.setPower(frontRightPower);
                bl.setPower(backLeftPower);
                br.setPower(backRightPower);
//                if (x!= 0) {
//                    if (y>=0) {
//                        rotation.setPosition(1 - (Math.acos(x / (Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5))) / Math.PI));
//                    }
//                }
                slideTarget += (y>0 && slideTarget<slideMax) ? slideInterval*y/1.5:0;
                slideTarget += (y<0 && slideTarget>500) ? slideInterval*y/1.5:0;
                if (gamepad1.left_trigger > 0 && rotationPos >=0 ) {
                    rotationPos -= gamepad1.left_trigger / 80;
                    if (rotationPos <0) rotationPos = 1; // Ensure upper bound
                }
                if (gamepad1.right_trigger > 0 && rotationPos <=1) {
                    rotationPos += gamepad1.right_trigger / 80;
                    if (rotationPos >1) rotationPos = 0; // Ensure lower bound
                }
                rotation.setPosition(rotationPos);
            }
//  ARM & SLIDE PID


//            armMotor.setPower((armTarget > 0 && armTarget < 700) ? armPIDF(armTarget, armMotor) : 0);
//            slideMotor.setPower((slideTarget > 60 && s
//            lideTarget < 800) ? slidePIDF(slideTarget, slideMotor) : 0);

            if (armTarget >= 0 && armTarget <= 2300) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            }else{
                armMotor.setPower(0);
            }
            if (slideTarget >= 200 && slideTarget <= slideMax) {
                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
            }else{
                slideMotor.setPower(0);
            }


            if (mode==Mode.INTAKING || micro){
                slideMax = 2900;
            }else{
                slideMax = 3000;
            }


//  CLAW
            if (gamepad1.a && mode != Mode.HANG) {
                if (!claw) {
                    claw=true;
                    clawOpen = !clawOpen;
                }
            } else {
                claw = false;
            }

            if (!clawOpen){
                clawR.setPosition(clawRClose);
            }else{
                clawR.setPosition(clawROpen);
            }

//  SLIDES
            slideTarget += (gamepad1.dpad_up && slideTarget<slideMax) ? slideInterval : 0;
            slideTarget -= (gamepad1.dpad_down && slideTarget>500) ? slideInterval : 0;
            slideTarget = Math.min(5300, Math.max(200, slideTarget));

            slideExtended = slideTarget > 300;

//  ARM

            armTempTarget += (gamepad1.left_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget -= (gamepad1.right_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget = Math.min(2300, Math.max(0, armTempTarget));

            armPar = (slideTarget > 300) ? 325 : 250;

//             /\_/\
//            ( o.o )
//             > ^ <    Purrrr...


//  MODES
            boolean rightBumperCurrentState = gamepad1.right_bumper;
            if (rightBumperCurrentState && !rightBumperPrevState) {
                if (mode == Mode.REST) {
                    mode = Mode.OUTTAKING;
                    slideInterval = 24;
                    init = true;
                } else if (mode == Mode.OUTTAKING) {
                    retractSlide=true;
                    slideTarget = 500;
                } else if (mode == Mode.INTAKING){
                    micro = false;
                    armTempTarget = armPar;
                    wrist.setPosition(wristPerp);
                    armTarget = armTempTarget;
//                    retractSlide = true;
                    slideTarget = 200;
                }
            }
            rightBumperPrevState = rightBumperCurrentState;
// RETRACT SLIDE
            if (retractSlide) {
                slideTarget = 200;
                if (slideMotor.getCurrentPosition() < 1800) {
                    retractSlide = false;
                    mode=Mode.REST;
                    init = true;
                }

            }


            telemetry.addData("retract",retractSlide);


            boolean hangCurr = gamepad1.left_stick_button;
            if (hangCurr && !hangPrev) {
                if (mode == Mode.REST) {
                    mode = Mode.HANG;
                } else if (mode == Mode.HANG) {
                    mode = Mode.REST;
                }
                init = true;
            }
            hangPrev = hangCurr;


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

// CHANGE TO INTAKING



                    if (slideTarget > 300) {
                        retracted = false;
                        mode = Mode.INTAKING;
                        init = true;
                    }

                    boolean intakeCurr = gamepad1.left_bumper;
                    if (intakeCurr && !intakePrev){
                        micro = true;
                        rotationPos = 0.5;
                        slideTarget = 1500;
                        mode = Mode.INTAKING;
                        init = true;
                    }
                    intakePrev = intakeCurr;

                    break;

/** INTAKING */
                case INTAKING:
                    if (init) {
                        wrist.setPosition(wristPar);
                        clawOpen = true;
                        armTempTarget = armPar;
                    }
                    init = false;


//  LOWER ARM
                    armTarget = (gamepad1.left_bumper) ? 100 : armTempTarget;

//  CHANGE TO REST
                    if (slideTarget <= 250){
                        mode = Mode.REST;
                        init = true;//retract slide < 70; rest <= 80; intaking > 80
                    }

                    break;

/** OUTTAKING */
                case OUTTAKING:
                    if (init) {
                        armTempTarget = armUp;
                        slideOuttake = true;
                        rotation.setPosition(0.5);



                    }
                    if (slideMotor.getCurrentPosition()>800){
                        wrist.setPosition(wristOuttake);
                    }
                    init = false;
                    slideTarget += (gamepad1.left_bumper && slideTarget<slideMax) ? slideInterval : 0;

                    if (slideOuttake && armTempTarget-armMotor.getCurrentPosition()<500){
                        slideTarget = 1600;
                        slideOuttake = false;
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
        return output;
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
        return output;
    }

}
