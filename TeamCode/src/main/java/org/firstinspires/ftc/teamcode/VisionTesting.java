package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.List;
import java.util.ArrayList;
import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Collections;

@TeleOp

public class VisionTesting extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvCamera webcam = null;
    public void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam,0);
        webcam.setPipeline(new pipeLine());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){

            @Override
            public void onOpened() {
                webcam.startStreaming(640,480,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });
    }
    @Override
    public void runOpMode(){
        initCamera();
        waitForStart();
        while (!isStopRequested()){
            dashboardTelemetry.addData("status","running");
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
                    dashboardTelemetry.addData("orientationtan", orientationtan);

             }
            }

            dashboardTelemetry.addData("contours detected", contours.size());
            dashboardTelemetry.update();

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


}