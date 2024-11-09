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


import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


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

    public double wristPar = 0, wristPerp = 0.55, wristOuttake = 0.8;
    public double clawLOpen = 1.0, clawLClose = 0.55, clawROpen = 0.0, clawRClose = 0.45;
    public double rotationPos = 0.5;
    public double armPar = 150, armUp = 2000;
    public int slideInterval = 15;
    public double hangClosed = 0.5, hangOpen = 1;

//  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.2, armI = 0, armD = 0.0017, armF = 0;
//    extended PID
    double armPE = 0.2, armIE = 0, armDE = 0.002, armFE = 0;
    double armTarget = 0.0;

//  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.1, slideI = 0, slideD = 0.0013, slideF = 0;
    double slidePE = 0.2, slideIE = 0, slideDE = 0, slideFE = 0;
    double slideTarget = 0.0;

    OpenCvCamera webcam = null;
    boolean sequence = false;
    boolean leftBumperPrevState = false;
    boolean rightBumperPrevState = false;
    boolean gamepad2RightBumperPrevState = false;
    boolean driver1yPrevState = false;
    boolean cameraOn = false;
    boolean claw = false;
    boolean clawOpen = false;
    boolean init = true;
    boolean slideExtended = false;
    boolean dpad = false;
    boolean bumpers = false;
    boolean retractSlide = false;
    boolean retracted = true;
    boolean xHold = false;
    boolean xPress = false;
    boolean slideOuttake = false;



    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double driver1Multiplier = 1;
    double driver2Multiplier = 0.25;
    double armTempTarget = armPar;
    double slideMax = 1800;
    double maxWrist;

    public enum Mode {
        REST,
        OUTTAKING,
        INTAKING,
        HANG
    }
    Mode mode = Mode.REST;


    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam,0);
        webcam.setPipeline(new pipeLine());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
                cameraOn = true;
            }

            @Override
            public void onError(int errorCode){

            }
        });
    }

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
        initCamera();


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

            if (armTarget >= 0 && armTarget <= 2500) {
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
                slideMax = 1800;
            }else{
                slideMax = 2800;
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
        slideTarget = Math.min(2800, Math.max(200, slideTarget));

        slideExtended = slideTarget > 300;

//  ARM

        armTempTarget += (gamepad1.left_trigger > 0) ? 3 : 0;
        armTempTarget -= (gamepad1.right_trigger > 0) ? 3 : 0;
        armTempTarget = Math.min(2500, Math.max(0, armTempTarget));

        armPar = (slideTarget > 300) ? 350 : 400;


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
            if (slideTarget > 200) {
                retracted = false;
                slideTarget -=15;
            } else {
                retracted = true;
                retractSlide = false;
                mode=Mode.REST;
                init = true;
            }

        }


        telemetry.addData("retract",retractSlide);

        if (gamepad2.x){

        }

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
                        webcam.stopStreaming();
                        wrist.setPosition(wristPerp);
                        slideTarget = 200;
                        armTempTarget = armPar;
                        webcam.stopStreaming();
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
                    boolean gamepad2RightBumperCurrentState = gamepad2.right_bumper;
                    if (gamepad2RightBumperCurrentState && !gamepad2RightBumperPrevState) {
                        cameraOn = !cameraOn;
                        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                            @Override
                            public void onOpened() {
                                if (cameraOn) {
                                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                                    FtcDashboard.getInstance().startCameraStream(webcam,0);
                                } else {
                                    webcam.stopStreaming();
                                }
                            }
                            @Override
                            public void onError(int errorCode) {}
                        });
                    }
                    gamepad2RightBumperPrevState = gamepad2RightBumperCurrentState;

                    if (!cameraOn) {
                        if (gamepad2.left_trigger > 0 && rotationPos < 1) {
                            rotationPos += gamepad2.left_trigger / 100;
                            if (rotationPos > 1) rotationPos = 1; // Ensure upper bound
                        }
                        if (gamepad2.right_trigger > 0 && rotationPos > 0) {
                            rotationPos -= gamepad2.right_trigger / 100;
                            if (rotationPos < 0) rotationPos = 0; // Ensure lower bound
                        }
                        rotation.setPosition(rotationPos);
                    }

//  LOWER ARM
                    armTarget = (gamepad2.left_bumper) ? 150 : armTempTarget;

//  CHANGE TO REST
                    if (slideTarget <= 250){
                        mode = Mode.REST; //retract slide < 70; rest <= 80; intaking > 80
                    }

                    break;

/** OUTTAKING */
                case OUTTAKING:
                    if (init) {
                        armTempTarget = 2000;
                        slideOuttake = true;
                        webcam.stopStreaming();
                        rotation.setPosition(0.5);
                        wrist.setPosition(wristPar);


                    }
                    init = false;

                    if (slideOuttake && armTempTarget-armMotor.getCurrentPosition()<50){
                        slideTarget = 1200;
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
                        armTarget = armPar + 50;
                        slideTarget = 100;
                        wrist.setPosition(wristPerp);
                        hang.setPosition(hangOpen);
                        rotation.setPosition(0.5);
                    }

//  ARM
                    armTarget = armTempTarget;

//                    TODO: test direction of motors are correct
                    if (gamepad1.a) {
                        hangL.setPower(0.5);
                        hangR.setPower(0.5);
                    } else if (gamepad1.b) {
                        hangL.setPower(-0.5);
                        hangR.setPower(-0.5);
                    } else {
                        hangL.setPower(0);
                        hangR.setPower(0);
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

    public class pipeLine extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input image to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the range for blue color in HSV
            Scalar lowerBlue = new Scalar(100, 150, 50); // lower bound of blue
            Scalar upperBlue = new Scalar(140, 255, 255); // upper bound of blue

            /* HSV FOR REFERENCE
            RED:
            lower bound = (0,100,100), upper bound = (10,255,255)
            lower bound = (160,100,100), upper bound = (180,255,255)

            YELLOW:
            lower bound = (20,100,100), upper bound = (30,255,255)

            BLUE:
            lower bound = (100,150,50), upper bound = (140,255,255)
             */

            // Threshold the HSV image to get only blue colors
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            // find contours in image
            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // finding quadrilaterals
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour)<500){
                    continue;
                }
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

                // approximate shape of contour lines
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                if (approxCurve.total() == 4) { // quadrilateral found
                    // Convert the approximation back to MatOfPoint for drawing
                    MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                    // draw shape onto webcam image
                    Imgproc.drawContours(input, Collections.singletonList(points), -1, new Scalar(0, 255, 0), 2);

                    double orientationtan = findOrientation(approxCurve.toArray());
                    telemetry.addData("orientationtan", orientationtan);
                    double servoDegree;
                    if (orientationtan>=90){
                        servoDegree = (orientationtan+90)/180;
                    }
                    else{
                        servoDegree = (orientationtan-90)/180;
                    }
                    telemetry.addData("servodegree",servoDegree);
                    if (cameraOn) {
                        rotation.setPosition((servoDegree));
                    }
                }
            }

            telemetry.update();

            return input;
        }

        private double findOrientation(Point[] points) {

            boolean shortest = false; //false for d1/d3 shortest, true for d2/d4 shortest
            //distances from each quadrilateral
            double d1 = Math.pow(Math.pow(points[0].x - points[1].x, 2) + Math.pow(points[0].y - points[1].y, 2), 0.5);
            double d2 = Math.pow(Math.pow(points[1].x - points[2].x, 2) + Math.pow(points[1].y - points[2].y, 2), 0.5);
            double d3 = Math.pow(Math.pow(points[2].x - points[3].x, 2) + Math.pow(points[2].y - points[3].y, 2), 0.5);
            double d4 = Math.pow(Math.pow(points[0].x - points[3].x, 2) + Math.pow(points[0].y - points[3].y, 2), 0.5);
            //find shortest
            if (d1 >= d2 && d3 >= d4) {
                shortest = true;
            }
            Point midBottom;
            Point midTop;
            //find shortest by comparing any two adjacent sides
            if (shortest) {
                midTop = new Point((points[0].x + points[3].x) / 2, (points[0].y + points[3].y) / 2);
                midBottom = new Point((points[1].x + points[2].x) / 2, (points[1].y + points[2].y) / 2);

            } else {
                midTop = new Point((points[0].x + points[1].x) / 2, (points[0].y + points[1].y) / 2);
                midBottom = new Point((points[2].x + points[3].x) / 2, (points[2].y + points[3].y) / 2);
            }
            //distances of x,y values between shortest sides
            double dx = midBottom.x - midTop.x;
            double dy = midBottom.y - midTop.y;
            //trig to calculate degree
            double orientation = Math.atan2(dy,dx) * (180.0/ Math.PI);
            //if angle is negative
            if (orientation<0){
                orientation += 360;
            }
            return 180-orientation;

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
        if (mode == mode.OUTTAKING){
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
