package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterCMD implements Subsystem {
    public static double KP = 0.0005;
    public static double KI = 0.0;
    public static double KD = 0.00001;
    public static double KF = 0.0002;

    public boolean isOneMan = false;

    public static double CR_KP = 0.00005;
    public static double CR_KI = 0.0;
    public static double CR_KD = 0.000000001;
    public static double CR_KF = 0.0001;

    public static double IDLE_SHOOTER = 1000;

    public static double TICKS_PER_REV = 28.0;
    public static double CR_TICKS_PER_REV = 28.0;
    public static double CLOSE_SHOOTER_RPM = 3450;

    public static double AUTO_SHOOTER_RPM = 3450;
    public static double AUTO_CR_RPM = 2650;
    public static double CLOSE_CR_RPM = 2700;
    public static double FAR_SHOOTER_RPM = 5100;
    public static double FAR_CR_RPM = 2600;
    public static double RPM_TOLERANCE = 50;

    public static double G = 386.1;
    public static double TARGET_Y = 25.5;
    public static double IMPACT_ANGLE_THETA = -0.002;
    public static double FLYWHEEL_RADIUS = 1.43701;

    public static double CR_RATIO = 0.7;
    public static double VELOCITY_TO_RPM_RATIO = 2.05;
    public static double RPM_BASE = 400.0;




    private double distanceToGoal = 0;

    public static double BlockerOpen = 0.55;

    public static double BlockerClosed = 1;

    Telemetry telemetry;

    private DcMotorEx shooterMotor;
    private DcMotorEx counterRoller;

    private Servo blocker;


    private double targetRPM = 0;
    private double crTargetRPM = 0;

    private double integral = 0;
    private double lastError = 0;

    private double crIntegral = 0;
    private double crLastError = 0;

    private long lastTime = 0;

    public PIDFController pidfController;
    public PIDFController CRpidfController;

    private Shooter.ShooterState state = Shooter.ShooterState.IDLE;
}

