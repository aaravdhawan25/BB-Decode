package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@TeleOp(name = "LockTo Test")
@Config
public class LockToTest extends LinearOpMode {

    MecanumDrive follower;

    Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

    public static double xyP = 0;
    public static double headingP = 0;
    public boolean isActive = false;

    public Pose2d lockTarget = null;
    public boolean wasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new MecanumDrive(hardwareMap, startPose);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){
            follower.updatePoseEstimate();
            if (gamepad1.triangleWasPressed()) {
                // Capture position once on press
                lockTarget = follower.localizer.getPose();
                isActive = true;
            }

            if (gamepad1.triangle) {  // While held (not wasPressed)
                // Keep running lockTo every loop
                lockTo(lockTarget);
                isActive = true;
            } if(gamepad1.triangleWasReleased()) {
                isActive = false;
            }

            if (isActive == false){
//                // Normal driving when not holding triangle
//                double y = -gamepad1.left_stick_y;
//                double x = gamepad1.left_stick_x;
//                double rx = gamepad1.right_stick_x;
//                follower.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(x, y),
//                        rx));
                drivetrain.update();
                drivetrain.updateCtrls(gamepad1,gamepad2);
            }


            telemetry.addData("Curr Pose", follower.localizer.getPose());
            telemetry.addData("xyP", xyP);
            telemetry.addData("headingP", headingP);
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
