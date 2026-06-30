package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.TWO_WHEEL;

        FollowerConstants.leftFrontMotorName = "LFM";
        FollowerConstants.leftRearMotorName = "LBM";
        FollowerConstants.rightFrontMotorName = "RFM";
        FollowerConstants.rightRearMotorName = "RBM";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 14.88;

        FollowerConstants.xMovement = 77.2472714478669;
        FollowerConstants.yMovement = 59.01833004718119;

        FollowerConstants.forwardZeroPowerAcceleration = -49.35493620534212;
        FollowerConstants.lateralZeroPowerAcceleration = -70.00019291650658;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.0002,0.02);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.02,0,0.0002,0); //  Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.9,0,0.02,0.01);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.5,0,0.15,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.06,0,0.000002,0,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.0006,0,0.0005,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0007;

        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.useVoltageCompensationInAuto = false;
        FollowerConstants.nominalVoltage = 12;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}