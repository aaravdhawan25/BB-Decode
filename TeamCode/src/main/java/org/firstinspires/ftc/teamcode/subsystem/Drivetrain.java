package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Drivetrain implements Subsystem {

    public DcMotor lfMotor, lbMotor, rfMotor, rbMotor;

    Telemetry telemetry;

    double y = 0, x = 0, rx = 0, denominator = 0;
    public double lfPower = 0, lbPower = 0, rfPower = 0, rbPower = 0;

    public boolean bumpToggle = false;

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
        lfMotor = map.get(DcMotor.class, "LFM");
        lbMotor = map.get(DcMotor.class, "LBM");
        rfMotor = map.get(DcMotor.class, "RFM");
        rbMotor = map.get(DcMotor.class, "RBM");

        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        lbMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.REVERSE);

        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
        telemetry.addData("Drivetrain", "Initialized");
        telemetry.update();
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

        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

        if (gp1.leftStickButtonWasPressed()) {
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
