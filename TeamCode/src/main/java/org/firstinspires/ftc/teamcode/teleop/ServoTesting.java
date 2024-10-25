package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends LinearOpMode {
    public Servo servo0 = null, servo1 = null ;
    public DcMotorEx motor3 = null;

    public double wristPar = 0.09, wristPerp = 0.64;
    public double clawLOpen = 1.0, clawLClose = 0.6, clawROpen = 0.0, clawRClose = 0.4;

    public void initHardware() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");


//        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        motor3.setPower(0);
    }

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (!isStopRequested()) {
//            telemetry.addData("motor3 pos", motor3.getCurrentPosition());
//            telemetry.update();
//
//            if (gamepad1.a) {
//                motor3.setPower(0.4);
//            }
//            else if (gamepad1.b) {
//                motor3.setPower(-0.4);
//            }
//            motor3.setPower(0);





            if (gamepad1.dpad_right) {
                servo0.setPosition(1);
            }
            else if (gamepad1.dpad_left) {
                servo0.setPosition(0);
            }
//
//            if (gamepad1.a){
//                servo0.setPosition(0.3);
//            }
//            else if (gamepad1.b) {
//                servo0.setPosition(0.5);
//            }
//            else if (gamepad1.x) {
//                servo0.setPosition(0.7);
//            }
//            else if (gamepad1.y) {
//                servo0.setPosition(1);
//            }



//            if (gamepad1.x) {
//                servo1.setPosition(0);
//            }
//            else if (gamepad1.y) {
//                servo1.setPosition(1);
//            }
        }
    }

}
