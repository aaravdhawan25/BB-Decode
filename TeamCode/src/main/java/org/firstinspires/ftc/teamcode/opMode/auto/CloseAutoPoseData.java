package org.firstinspires.ftc.teamcode.opMode.auto;

import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.finalShootXCont;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.finalShootYCont;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.gateReturnXC;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.gateReturnYC;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.intakeSpike1X;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.intakeSpike3ContX;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.intakeSpike3ContY;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.intakeSpike3Y;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.leverIntakeX;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.leverIntakeY;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.leverX;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.leverYNew;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.startHeading;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants;

@Config
public class CloseAutoPoseData {
    public static final Pose LEVER_SECOND     = new Pose(leverX, leverYNew);

    public static final Pose START_POSE     = new Pose(startX, startY, Math.toRadians(startHeading));
    public static final Pose SHOOTING_POSE  = new Pose(AutoConstants.shootingX, AutoConstants.shootingY);
    public static final Pose MID2_CURVE     = new Pose(AutoConstants.MID_2x, AutoConstants.MID_2y);
    public static final Pose SECOND_INTAKE  = new Pose(AutoConstants.secondIntakeX, AutoConstants.secondIntakeY);
    public static final Pose LEVER = new Pose(leverX, AutoConstants.leverY);

    public static final Pose INTAKE1 = new Pose(intakeSpike1X, AutoConstants.intakeSpike1Y);

    public static final Pose INTAKE3Cont = new Pose(intakeSpike3ContX, intakeSpike3ContY);

    public static final Pose finalShootCont = new Pose(finalShootXCont, finalShootYCont);

    public static final Pose LEVER_RETURN_CONTROL = new Pose(gateReturnXC, gateReturnYC);
    public static final Pose LEVER_CONTROL = new Pose(AutoConstants.leverPoseX, AutoConstants.leverPoseY);
    public static final Pose FINAL_INTAKE   = new Pose(AutoConstants.intakeSpike3X, intakeSpike3Y);
    public static final Pose FINAL_SHOOT   = new Pose(63.253,  100.482);
    public static final Pose LEVER_INTAKE = new Pose(leverIntakeX, leverIntakeY);
    public static double mirrorX(double x, String color) {
        return color.equals("RED") ? 144 - x : x;
    }
    public static double mirrorHeading(double deg, String color) {
        return color.equals("RED") ? 180 - deg : deg;
    }
    public static Pose mirror(Pose p, String color) {
        return color.equals("RED") ? new Pose(144 - p.getX(), p.getY(), Math.toRadians(180 - Math.toDegrees(p.getHeading()))) : p;
    }
}
