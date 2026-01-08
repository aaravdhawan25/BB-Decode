package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;
import org.firstinspires.ftc.teamcode.subsystem.New.Turret;

@TeleOp(name = "TurretTeleRed", group = "Blue")
@Disabled
public class prod_TestTele_Red extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(0,-48.8,Math.toRadians(180));
    Drivetrain drivetrain;
    Turret turret;
    Intaker intaker;
    Shooter2 shooter;

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap,telemetry);
        turret = new Turret(hardwareMap, telemetry);
        intaker = new Intaker(hardwareMap, telemetry);
        shooter = new Shooter2(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, START_POSE);
        drivetrain.init();
        turret.init();
        intaker.init();
        shooter.init();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());


        while (!opModeIsActive()) {
            turret.updatePose(START_POSE.position, Math.toDegrees(START_POSE.heading.toDouble()));
            turret.returnTurretHome();
        }

        waitForStart();

        while (opModeIsActive()){

            drive.updatePoseEstimate();

            // Get current pose from Road Runner
            Pose2d currentPose = drive.localizer.getPose();
            Vector2d currentPos = currentPose.position;
            double currentHeading = Math.toDegrees(currentPose.heading.toDouble());

            turret.updatePose(currentPos, currentHeading);

            turret.update();
            drivetrain.update();
            intaker.update();
            shooter.update();
            drivetrain.updateCtrls(gamepad1, gamepad2);
            turret.updateCtrls(gamepad1, gamepad2);
            intaker.updateCtrls(gamepad1, gamepad2);
            shooter.updateCtrls(gamepad1, gamepad2);

            telemetry.update();
        }
    }
}
