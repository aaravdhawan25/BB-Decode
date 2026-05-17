package org.firstinspires.ftc.teamcode.subsystem.New;


import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.MAX_STEP_PER_LOOP;
import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.P;
import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.SLOPE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.localization.Pose;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

public class TurretCMD implements Subsystem {

    Servo turretServo1, turretServo2;

    public TurretState state;

    private double turretPose = TurretConstants.turretForwardPosition;

    public TurretCMD(Servo turretServo1, Servo turretServo2){
        this.turretServo1 = turretServo1;
        this.turretServo2 = turretServo2;
    }

    public void setState(TurretState state) {
        if (state == null) return;
        this.state = state;
        switch (state) {
            case FORWARD:
                setServoPos(TurretConstants.turretForwardPosition);
                break;
            case MATH:
                pointToGoalPinPoint(Robot.robotPos);
                break;

            case WHILE_MOVING_TURRET:
                pointToGoalVelocityCompensated(Robot.robotPos);

        }
    }

    private void setServoPos(double position) {
        position = Range.clip(position, 0, 1);
        turretPose = position;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

//    private void pointToGoalCamera(LLCam.TagTarget tag){
//        if (tag == null || !tag.hasTarget) return;
//        double tX = tag.tX;
//        if (Math.abs(tX) < 0.5){
//            return;
//        }
//        double deltaPos = tX * SLOPE * P;
//        double targetAngleDeg = turretPose - deltaPos;
//
//        double step = Range.clip(targetAngleDeg - turretPose, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
//        setServoPos(turretPose + step);
//
//
//        PerTelem.addData("Math Camera", true);
//    }

    private void pointToGoalPinPoint(Pose robotPos) {
        Vector2d goal = Robot.getGoalPos();
        double robotHeadingRad = Math.toRadians(robotPos.getHeading());
        Vector2d turretPos = new Vector2d(
                robotPos.getX() + TurretConstants.turretOffsetInchesx * Math.cos(robotHeadingRad) - TurretConstants.turretOffsetInchesy * Math.sin(robotHeadingRad),
                robotPos.getY() + TurretConstants.turretOffsetInchesx * Math.sin(robotHeadingRad) + TurretConstants.turretOffsetInchesy * Math.cos(robotHeadingRad)
        );
        Vector2d toGoal = goal.minus(turretPos);
        double fieldAngle = Math.atan2(
                toGoal.y, toGoal.x
        );
//        double relAngle = fieldAngle - robotPos.heading.toDouble() - Math.PI;
        double relAngle = fieldAngle - robotHeadingRad;
        while (relAngle > Math.PI)  relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        double angleDeg = Math.toDegrees(relAngle);
        double servoPos = TurretConstants.OFFSET + SLOPE * angleDeg;
        if (servoPos > 1.0) servoPos = 1.0;
        if (servoPos < 0.0) servoPos = 0.0;
        setServoPos(servoPos);
        PerTelem.addData("Turret Servo Position", servoPos);
        PerTelem.addData("Turret Angle", angleDeg);
    }

//    private void pointToGoalVelocityCompensated(Pose2d robotPos) {
//        Vector2d goal = Robot.getGoalPos();
//
//        double robotHeadingRad = robotPos.heading.toDouble();
//        Vector2d turretPos = new Vector2d(
//                robotPos.position.x + TurretConstants.turretOffsetInchesx * Math.cos(robotHeadingRad) - TurretConstants.turretOffsetInchesy * Math.sin(robotHeadingRad),
//                robotPos.position.y + TurretConstants.turretOffsetInchesx * Math.sin(robotHeadingRad) + TurretConstants.turretOffsetInchesy * Math.cos(robotHeadingRad)
//        );
//
//        Vector2d toGoal = goal.minus(turretPos);
//        double distanceToGoal = Math.max(toGoal.norm(), 1.0);
//        double fieldAngle = Math.atan2(toGoal.y, toGoal.x);
//
//        PoseVelocity2d poseVel = Robot.robotVel;
//        double velocityMagnitude = Math.hypot(poseVel.linearVel.x, poseVel.linearVel.y);
//
//        double turretVelCompOffset = 0.0;
//        if (velocityMagnitude > 0.5) {
//            double velocityTheta = Math.atan2(poseVel.linearVel.y, poseVel.linearVel.x);
//
//            double coordinateTheta = velocityTheta - fieldAngle;
//            double parallelComponent = -Math.cos(coordinateTheta) * velocityMagnitude;
//            double perpendicularComponent = Math.sin(coordinateTheta) * velocityMagnitude;
//
//            double timeOfFlight = distanceToGoal * TurretConstants.FLIGHT_TIME_PER_INCH;
//            double ivr = distanceToGoal / timeOfFlight + parallelComponent;
//
//            turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
//        }
//
//        double compensatedFieldAngle = fieldAngle + turretVelCompOffset;
//        double relAngle = compensatedFieldAngle - robotPos.heading.toDouble() - Math.PI;
//        while (relAngle > Math.PI)  relAngle -= 2 * Math.PI;
//        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//
//        double angleDeg = Math.toDegrees(relAngle);
//        double servoPos = TurretConstants.OFFSET + SLOPE * angleDeg;
//        servoPos = Range.clip(servoPos, 0, 1.0);
//
//        setServoPos(servoPos);
//
//        PerTelem.addData("Turret Servo Position (VelComp)", servoPos);
//        PerTelem.addData("Turret Angle (VelComp)", angleDeg);
//        PerTelem.addData("Vel Comp Offset (deg)", Math.toDegrees(turretVelCompOffset));
//    }

    private void pointToGoalVelocityCompensated(Pose robotPos) {
        Vector2d goal = Robot.getGoalPos();

        // robotPos.heading.toDouble() returns degrees in RoadRunner, must convert
        // double robotHeadingRad = robotPos.heading.toDouble(); // OLD - wrong, degrees not radians
        double robotHeadingRad = Math.toRadians(robotPos.getHeading());

        Vector2d turretPos = new Vector2d(
                robotPos.getX() + TurretConstants.turretOffsetInchesx * Math.cos(robotHeadingRad) - TurretConstants.turretOffsetInchesy * Math.sin(robotHeadingRad),
                robotPos.getY() + TurretConstants.turretOffsetInchesx * Math.sin(robotHeadingRad) + TurretConstants.turretOffsetInchesy * Math.cos(robotHeadingRad)
        );

        Vector2d toGoal = goal.minus(turretPos);
        double distanceToGoal = Math.max(toGoal.norm(), 1.0);
        double fieldAngle = Math.atan2(toGoal.y, toGoal.x);

        PoseVelocity2d poseVel = Robot.robotVel;
        double velocityMagnitude = Math.hypot(poseVel.linearVel.x, poseVel.linearVel.y);

        double turretVelCompOffset = 0.0;
        if (velocityMagnitude > 0.5) {
            double velocityTheta = Math.atan2(poseVel.linearVel.y, poseVel.linearVel.x);

            double coordinateTheta = velocityTheta - fieldAngle;

            // removed negative sign - moving toward goal should increase closing speed
            // double parallelComponent = -Math.cos(coordinateTheta) * velocityMagnitude; // OLD - sign was inverted
            double parallelComponent = Math.cos(coordinateTheta) * velocityMagnitude;
            double perpendicularComponent = Math.sin(coordinateTheta) * velocityMagnitude;

            // distanceToGoal / timeOfFlight simplifies to 1.0 / FLIGHT_TIME_PER_INCH since they cancel
            // double timeOfFlight = distanceToGoal * TurretConstants.FLIGHT_TIME_PER_INCH; // OLD - distance cancels out
            // double ivr = distanceToGoal / timeOfFlight + parallelComponent;              // OLD - unclear
            double projectileSpeed = 1.0 / TurretConstants.FLIGHT_TIME_PER_INCH;
            double ivr = projectileSpeed + parallelComponent;

            turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
        }

        double compensatedFieldAngle = fieldAngle + turretVelCompOffset;

        // robotPos.heading.toDouble() is degrees, must use robotHeadingRad
        // double relAngle = compensatedFieldAngle - robotPos.heading.toDouble() - Math.PI; // OLD - mixed radians and degrees
        double relAngle = compensatedFieldAngle - robotHeadingRad - Math.PI;

        while (relAngle > Math.PI)  relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

        double angleDeg = Math.toDegrees(relAngle);
        double servoPos = TurretConstants.OFFSET + SLOPE * angleDeg;
        servoPos = Range.clip(servoPos, 0, 1.0);

        setServoPos(servoPos);

        PerTelem.addData("Turret Servo Position (VelComp)", servoPos);
        PerTelem.addData("Turret Angle (VelComp)", angleDeg);
        PerTelem.addData("Vel Comp Offset (deg)", Math.toDegrees(turretVelCompOffset));
    }

    public enum TurretState {
        FORWARD, MATH, MATH_CAMERA, WHILE_MOVING_TURRET
    }

}
