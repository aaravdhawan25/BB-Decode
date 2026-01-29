package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DistanceToGoal implements Subsystem {
    Telemetry telemetry;

    public Vector2d robotPos = new Vector2d(0, 0);
    private double robotHeading = 0;
    private Vector2d turretPos = new Vector2d(0,0);
    private static double turretOffsetInchesx = -5;
    private static double turretOffsetInchesy = -2;

    public DistanceToGoal(Telemetry telemetry){
        this.telemetry = telemetry;

    }

    @Override
    public void init() {

    }

    public void updatePose(Vector2d pos, double headingDeg) {
        this.robotPos = pos;
        this.robotHeading = headingDeg;

        double robotHeadingRad = Math.toRadians(headingDeg);

        turretPos = new Vector2d(
                robotPos.x - turretOffsetInchesx * Math.cos(robotHeadingRad),
                robotPos.y - turretOffsetInchesy * Math.sin(robotHeadingRad)
        );
    }

    public double calculateDistanceToGoal(Vector2d robotPos, Vector2d goalPos) {
        Vector2d toGoal = goalPos.minus(robotPos);
        return Math.sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);
    }

    public double calculateRobotAngle(Vector2d robotPos, double robotHeadingDeg, Vector2d goalPos) {
        Vector2d toGoal = goalPos.minus(robotPos);
        double angleToGoal = Math.toDegrees(Math.atan2(toGoal.y, toGoal.x));
        angleToGoal = AngleUnit.normalizeDegrees(angleToGoal) + 180;
//        if (angleToGoal < 0) {
//            angleToGoal += 360;
//        }
//        double normalizedHeading = robotHeadingDeg;
//        if (normalizedHeading < 0) {
//            normalizedHeading += 360;
//        }
        double robotAngle = AngleUnit.normalizeDegrees(angleToGoal - robotHeadingDeg);
//        while (turretAngle > 180) {
//            turretAngle -= 360;
//        }
//        while (turretAngle < -180) {
//            turretAngle += 360;
//        }
        return robotAngle;
    }
    @Override
    public void update() {
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

    }
}
