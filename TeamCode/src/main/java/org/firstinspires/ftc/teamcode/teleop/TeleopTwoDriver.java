package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

@TeleOp
public class TeleopTwoDriver extends LinearOpMode {
    DcMotorEx armMotor,fl,fr,bl,br = null;
//    DcMotorEx slidesMotor = null;

    public void initHardware() {
        //armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        fl = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class,"backRightMotor");
//        sl idesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);


//        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


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

            telemetry.update();

        }
    }

}
