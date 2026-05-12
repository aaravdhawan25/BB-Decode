package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.LLCam;

@Config
public class Drivetrain implements Subsystem {

    Telemetry telemetry;

    double y = 0, x = 0, rx = 0, denominator = 0;
    public static double lfPower = 0, lbPower = 0, rfPower = 0, rbPower = 0;

    public static DcMotorEx lbMotor, lfMotor, rfMotor, rbMotor;

    public boolean bumpToggle = false;

    public boolean isDrive = false;

    public static double lfDP = 1;
    public static double lbDP = 1;
    public static double rfDP = -1;
    public static double rbDP = -1;

    public static double lfStrafe = 1;
    public static double lbStrafe = 1;
    public static double rfStrafe = -1;
    public static double rbStrafe = -1;
    public Drivetrain(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        lfMotor = map.get(DcMotorEx.class, "LFM");
        lbMotor = map.get(DcMotorEx.class, "LBM");
        rfMotor = map.get(DcMotorEx.class, "RFM");
        rbMotor = map.get(DcMotorEx.class, "RBM");


    }

    @Override
    public void init() {
        lbMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rfMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rbMotor.setDirection(DcMotorEx.Direction.REVERSE);

        lfMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Drivetrain", "Initialized");
        telemetry.update();
        lfMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void setDriveVectors(double y, double x, double rx) {
        this.y = y;
        this.x = x;
        this.rx = rx;
    }

    @Override
    public void update() {
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lfPower = (y + x + rx) / denominator;
        lbPower = (y - x + rx) / denominator;
        rfPower = (y - x - rx) / denominator;
        rbPower = (y + x - rx) / denominator;
        if (LLCam.getState() == LLCam.alignState.MATH_CAMERA) {
            lfMotor.setPower(-LLCam.rotationalPower);
            lbMotor.setPower(-LLCam.rotationalPower);
            rfMotor.setPower(LLCam.rotationalPower);
            rbMotor.setPower(LLCam.rotationalPower);
        } else if (LLCam.getState() == LLCam.alignState.STOP){
            lfMotor.setPower(lfPower);
            lbMotor.setPower(lbPower);
            rfMotor.setPower(rfPower);
            rbMotor.setPower(rbPower);
        }
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

        if (gp1.touchpadWasPressed()) {
            bumpToggle = !bumpToggle;
        }

        if (bumpToggle) {
            setDriveVectors(-gp1.left_stick_y * 0.3, gp1.left_stick_x * 0.4, gp1.right_stick_x * 0.3);
        } else {
            setDriveVectors(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);
        }

        update();
    }

}
