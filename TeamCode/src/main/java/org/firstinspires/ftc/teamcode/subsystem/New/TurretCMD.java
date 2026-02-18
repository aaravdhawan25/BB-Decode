package org.firstinspires.ftc.teamcode.subsystem.New;


import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.MAX_STEP_PER_LOOP;
import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.P;
import static org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants.SLOPE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
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
                pointToGoalPinPoint(Robot.currPos);
                break;
        }
    }

    private void setServoPos(double position) {
        position = Range.clip(position, 0, 1);
        turretPose = position;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    private void pointToGoalPinPoint(Pose2d robotPos) {
        Vector2d goal = Robot.getGoalPos();
        double robotHeadingRad = Math.toRadians(robotPos.heading.toDouble());
        Vector2d turretPos = new Vector2d(
                robotPos.position.x- TurretConstants.turretOffsetInchesx * Math.cos(robotHeadingRad),
                robotPos.position.y - TurretConstants.turretOffsetInchesy * Math.sin(robotHeadingRad)
        );
        Vector2d toGoal = goal.minus(turretPos);
        double fieldAngle = Math.atan2(
                toGoal.y, toGoal.x
        );
        double relAngle = fieldAngle - robotPos.heading.toDouble() - Math.PI;
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

    public enum TurretState {
        FORWARD, MATH, MATH_CAMERA
    }

}
