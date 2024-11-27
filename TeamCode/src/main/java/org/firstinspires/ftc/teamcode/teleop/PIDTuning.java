package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
@Config
public class PIDTuning extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DcMotorEx armMotor;
    public DcMotorEx slideMotor;

    public final int armParallel = 30, armUp = 550;

    //    Arm PID
    public static PIDFController armPIDF = new PIDFController(0,0,0, 0);
    public static double armP = 0.2, armI = 0, armD = 0.002, armF = 0;
//    armP is 0.025 when slide is out
    public static double armTarget = 0.0;

    //    Slide PID
    public static PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public static double slideP = 0.05, slideI = 0, slideD = 0, slideF = 0;
    public static double slideTarget = 0.0;

    public void initHardwware() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

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
        initHardwware();
        waitForStart();

        while (!isStopRequested()) {
            if (armTarget > 0 && armTarget < 3000) {
                armMotor.setPower(armPIDF(armTarget, armMotor));
            }
            if (slideTarget > 70 && slideTarget < 7000) {
                slideMotor.setPower(slidePIDF(slideTarget, slideMotor));
            }

        }
    }

    public double armPIDF(double target, DcMotorEx motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        dashboardTelemetry.addData("arm current position: ", currentPosition);
        dashboardTelemetry.addData("arm target: ", target);
        dashboardTelemetry.update();
        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        slidePIDF.setPIDF(slideP,slideI,slideD,slideF);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        dashboardTelemetry.addData("slide current position: ", currentPosition);
        dashboardTelemetry.addData("slide target: ", target);
        dashboardTelemetry.update();
        return output/8;
    }

}
