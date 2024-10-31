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
    DcMotorEx armMotor,fl,fr,bl,br = null;
    Servo servoArm,wrist,clawL,clawR;
//    SERVO VALUES
//    not reversed
//     range = 0, 0.74
//    parallel = 0
//    perpendicular = 0.55

//    DcMotorEx slidesMotor = null;
    OpenCvCamera webcam = null;
    public static PIDFController armPIDF = new PIDFController(0,0,0, 0);
    public static double armP = 0.0022, armI = 0.01, armD = 0.00008, armF = 0;
    //    armP is 0.025 when slide is out
    public static double armTarget = 0.0;

    //    Slide PID
    public static PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public static double slideP = 0.05, slideI = 0, slideD = 0, slideF = 0;
    public static double slideTarget = 0.0;


    public void initHardware() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam,0);
        webcam.setPipeline(new pipeLine());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

            }
        });
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        fl = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        servoArm = hardwareMap.get(Servo.class,"Servo1");
        wrist = hardwareMap.get(Servo.class,"wrist");
        clawL = hardwareMap.get(Servo.class,"clawL");
        clawR = hardwareMap.get(Servo.class,"clawR");
//        sl idesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        wrist.setPosition(0);
        clawL.setPosition(1);
        clawR.setPosition(0.04);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setPower(0);


    }

    @Override
    public void runOpMode() {

        initHardware();
        waitForStart();


        while (!isStopRequested()) {
            //armMotor.setPower(0.2);

            telemetry.addData("status","running");
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);
            double frontLeftPower = (y+x+rx)/denom;
            double backLeftPower = (y-x+rx)/denom;
            double frontRightPower = (y-x-rx)/denom;
            double backRightPower =  (y+x-rx)/denom;
            telemetry.addData("fl",frontLeftPower);
            telemetry.addData("fr",frontRightPower);
            telemetry.addData("bl",backLeftPower);
            telemetry.addData("br",backRightPower);

            fl.setPower(frontLeftPower);
            fr.setPower(frontRightPower);
            bl.setPower(backLeftPower);
            br.setPower(backRightPower);

            if (armTarget > 0 && armTarget < 700) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            }

            if (gamepad1.left_bumper){
                armTarget += 0.75;
            }else if (gamepad1.right_bumper){
                armTarget -= 0.75;
            }
            telemetry.addData("arm",armTarget);


            // arm degree; todo - use settargetposition instead of setpower
//            if (gamepad1.left_bumper){
//                armMotor.setPower(-1);
////            }else if (gamepad1.right_bumper){
////                armMotor.setPower(1);
//            }else{
//                armMotor.setPower(0);
//            }
//            int pos = armMotor.getCurrentPosition();
//            telemetry.addData("pos",pos);

            //    SERVO VALUES
        //    not reversed
        //     range = 0, 0.74
        //    parallel = 0
        //    perpendicular = 0.55

            if (gamepad1.x){
                wrist.setPosition(0.55);
            } else if (gamepad1.y){
                wrist.setPosition(0);
            }
            telemetry.update();

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
                        servoDegree = (orientationtan-90)/180;
                    }
                    else{
                        servoDegree = (orientationtan+90)/180;
                    }
                    telemetry.addData("servodegree",servoDegree);
                    servoArm.setPosition((servoDegree));

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
