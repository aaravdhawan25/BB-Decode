package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@TeleOp(name = "LockTo Test")
@Config
public class LockToTest extends LinearOpMode {

    MecanumDrive follower;

    Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

    double xyP = 0;
    double headingP = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        while (opModeIsActive()){
            follower.updatePoseEstimate();
            if (gamepad1.triangleWasPressed()){
                lockTo(follower.localizer.getPose());
            }

            telemetry.addData("Curr Pose", follower.localizer.getPose());
            telemetry.update();

        }

    }

//    public void lockTo(Pose2d targetPos) {
//        Pose2d currPos = follower.localizer.getPose();
//
//        Vector2d difference = targetPos.position.minus(currPos.position);
//        Vector2d xy = difference.rotated(-currPos.heading.toDouble());
//
//        double heading = targetPos.heading.toDouble() - currPos.heading.toDouble();
//        // Normalize heading to [-PI, PI]
//        heading = Rotation2d.exp(heading).toDouble();
//
//        follower.setDrivePowers(new PoseVelocity2d(
//                xy.times(xyP),
//                heading * headingP
//        ));
//    }

    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = follower.localizer.getPose();

        Vector2d difference = targetPos.position.minus(currPos.position);

        // Manually rotate the difference vector by -currPos.heading
        double angle = -currPos.heading.toDouble();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        Vector2d xy = new Vector2d(
                difference.x * cos - difference.y * sin,
                difference.x * sin + difference.y * cos
        );

        double heading = targetPos.heading.toDouble() - currPos.heading.toDouble();
        // Normalize heading to [-PI, PI]
        heading = Rotation2d.exp(heading).toDouble();

        follower.setDrivePowers(new PoseVelocity2d(
                xy.times(xyP),
                heading * headingP
        ));
    }


}
