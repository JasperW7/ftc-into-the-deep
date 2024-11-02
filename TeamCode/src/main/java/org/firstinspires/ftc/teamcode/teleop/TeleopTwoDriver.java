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
    DcMotorEx armMotor, slideMotor, fl, fr, bl, br = null;
    Servo rotation, wrist, clawL, clawR;

    public double wristPar = 0.0, wristPerp = 0.55;
    public double clawLOpen = 1.0, clawLClose = 0.55, clawROpen = 0.0, clawRClose = 0.45;
    public double rotationPos = 0;

//  ARM PID
    public static PIDFController armPIDF = new PIDFController(0,0,0, 0);
    public static double armP = 0.0022, armI = 0.01, armD = 0.00008, armF = 0;
    //    armP is 0.025 when slide is out
    public static double armTarget = 0.0;

//  SLIDES PID
    public static PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public static double slideP = 0.05, slideI = 0, slideD = 0, slideF = 0;
    public static double slideTarget = 0.0;

    OpenCvCamera webcam = null;
    boolean cameraOn = false;

    boolean dpad = false;
    boolean bumpers = false;
    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double driver1Multiplier = 0.8;
    double driver2Multiplier = 0.15;

    public enum Mode {
        REST,
        OUTTAKING,
        HANG // Maybe
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

        rotation = hardwareMap.get(Servo.class,"rotation");
        wrist = hardwareMap.get(Servo.class,"wrist");
        clawL = hardwareMap.get(Servo.class,"clawL");
        clawR = hardwareMap.get(Servo.class,"clawR");

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        wrist.setPosition(wristPerp);
        clawL.setPosition(clawLOpen);
        clawR.setPosition(clawROpen);
        rotation.setPosition(0);

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

//            telemetry.addData("fl",frontLeftPower);
//            telemetry.addData("fr",frontRightPower);
//            telemetry.addData("bl",backLeftPower);
//            telemetry.addData("br",backRightPower);

            fl.setPower(frontLeftPower);
            fr.setPower(frontRightPower);
            bl.setPower(backLeftPower);
            br.setPower(backRightPower);
//            if (gamepad1.dpad_left){
//                slideTarget+= 0.5;
//            }else if (gamepad1.dpad_right){
//                slideTarget -=0.5;
//            }

//  ARM & SLIDE PID
            if (armTarget >= 0 && armTarget <= 700) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            }
            if (slideTarget >= 70 && slideTarget <= 800) {
                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
            }

//  MODES
//            if (gamepad1.left_bumper && gamepad1.right_bumper) {
//                if (!bumpers) {
//                    bumpers=true;
//                    if (mode == Mode.REST) {
//                        mode = Mode.OUTTAKING;
//                    } else if (mode == Mode.OUTTAKING) {
//                        mode = Mode.REST;
//                    }
//                }
//            } else {
//                bumpers = false;
//            }
            dashboardTelemetry.addData("mode type",mode);
            switch (mode) {
                case REST:
                    dashboardTelemetry.addData("mode", "REST");
                    armTarget = 0;
//                    arm down

                case OUTTAKING:
                    dashboardTelemetry.addData("mode", "OUTTAKING");
                    armTarget = 100;

//                    arm up
//                    slides at half when right trigger
//                    slides at full when left trigger

            }
//  ARM
//            if (gamepad1.left_bumper){
//                armTarget += 0.75;
//            } else if (gamepad1.right_bumper){
//                armTarget -= 0.75;
//            }
//            telemetry.addData("arm",armTarget);

//  ROTATION
            if (gamepad1.dpad_down){
                if (!dpad) {
                    dpad = true;

                    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
                        @Override
                        public void onOpened(){
                            if (cameraOn) {
                                webcam.setPipeline(new pipeLine());
                                webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
                            } else {
                                webcam.stopStreaming();
                            }
                        }
                        @Override
                        public void onError(int errorCode){}
                    });

                    cameraOn = !cameraOn;
                }
            }
            else {
                dpad = false;
            }

            if (!cameraOn && mode==Mode.REST) {
                if (rotationPos <= 1 && gamepad1.left_trigger > 0) {
                    rotationPos += gamepad1.left_trigger / 100 ;
                } else if (rotationPos >= 0 && gamepad1.right_trigger > 0) {
                    rotationPos -= gamepad1.right_trigger / 100 ;
                }
                rotation.setPosition(rotationPos);
            }
            if (gamepad1.a && armTarget<700){
                armTarget += 0.01;
            }else if (gamepad1.b && armTarget>0){
                armTarget -=0.01;
            }

//  WRIST
            if (gamepad1.x){
                wrist.setPosition(wristPar);
            } else if (gamepad1.y){
                wrist.setPosition(wristPerp);
            }

//  CLAW
            if (gamepad1.left_stick_button){
                clawL.setPosition(clawLOpen);
                clawR.setPosition(clawROpen);
            } else if (gamepad1.right_stick_button){
                clawL.setPosition(clawLClose);
                clawR.setPosition(clawRClose);
            }
            dashboardTelemetry.addData("arm",armMotor.getCurrentPosition());
            dashboardTelemetry.addData("arm target",armTarget);
            dashboardTelemetry.addData("slide current",slideMotor.getCurrentPosition());
            dashboardTelemetry.addData("slide target",slideTarget);

            dashboardTelemetry.addData("camera",cameraOn);
            dashboardTelemetry.addData("rotation",rotationPos);
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
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        telemetry.addData("arm current position: ", currentPosition);
        telemetry.addData("arm target: ", target);
        telemetry.update();
        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        slidePIDF.setPIDF(slideP,slideI,slideD,slideF);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        telemetry.addData("slide current position: ", currentPosition);
        telemetry.addData("slide target: ", target);
        telemetry.update();
        return output/8;
    }

}
