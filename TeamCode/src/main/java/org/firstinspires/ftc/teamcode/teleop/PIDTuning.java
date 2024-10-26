package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;
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

    //    Arm PID
    public static PIDController armPID = new PIDController(0,0,0);
    public static double armP = 0.0038, armI = 0, armD = 0.0002;
    public static double armTarget = 0.0;

    //    Slide PID
    public static PIDController slidePID = new PIDController(0,0,0);
    public static double slideP = 0, slideI = 0, slideD = 0;
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
            armMotor.setPower(armPID(armTarget, armMotor));
            slideMotor.setPower(slidePID(slideTarget, slideMotor));
        }
    }

    public double armPID(double target, DcMotorEx motor){
        armPID.setPID(armP,armI,armD);
        int currentPosition = motor.getCurrentPosition();
        double output = armPID.calculate(currentPosition, target);

        dashboardTelemetry.addData("arm current position: ", currentPosition);
        dashboardTelemetry.addData("arm target: ", target);
        dashboardTelemetry.update();
        return output;
    }

    public double slidePID(double target, DcMotorEx motor){
        slidePID.setPID(slideP,slideI,slideD);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePID.calculate(currentPosition, target);

        dashboardTelemetry.addData("slide current position: ", currentPosition);
        dashboardTelemetry.addData("slide target: ", target);
        dashboardTelemetry.update();
        return output;
    }

}
