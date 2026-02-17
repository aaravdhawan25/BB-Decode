package org.firstinspires.ftc.teamcode.utils.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
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
    public static double distanceToGoal = 0;
    public static double targetRPM = 0;
    public static double crTargetRPM = 0;
    public static double integral = 0;
    public double lastError = 0;
    public double crIntegral = 0;
    public double crLastError = 0;
    public long lastTime = 0;

    public static double MAX_RPM = 6000;

    public static double tuningRPM = 2000;
    


}
