package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.DistanceToGoal;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;
import org.opencv.core.Mat;

@Config
@TeleOp(name = "COMP Teleop", group = "a")
public class prod_TestTele extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-40.4,-20, Math.toRadians(245));



    Drivetrain drivetrain;
    Intaker intaker;
    Shooter2 shooter;

    DistanceToGoal distanceToGoal;

    ElapsedTime commandTime = new ElapsedTime();

    boolean isEndgame = false;

    public static Vector2d goalPos = new Vector2d(-70,-70);

    Vector2d robotPos = new Vector2d(0,0);

    double robotHeading = 0;
    public static double FarShootDelay1 = 0.6;

    public static double FarShootDelayDelay = 1.6;

    public static double FarShootDelay2Delay = 0.6;

    public static double FarShootDelay2 = FarShootDelay1+0.05 + FarShootDelay1;

    public double distance = 0;


    boolean set = false;

    boolean set2 = false;


    MecanumDrive follower;
    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime FarShoot = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap,telemetry);
        intaker = new Intaker(hardwareMap, telemetry);
        shooter = new Shooter2(hardwareMap, telemetry);
        follower = new MecanumDrive(hardwareMap, START_POSE);
        distanceToGoal = new DistanceToGoal(telemetry);
        drivetrain.init();
        intaker.init();
        shooter.init();


        while (opModeInInit()) {
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
            runtime.reset();
            commandTime.reset();
            FarShoot.reset();

        }

        waitForStart();

        while (opModeIsActive()){
            runtime.startTime();
            commandTime.startTime();
            FarShoot.startTime();
            robotPos = follower.localizer.getPose().position;
            follower.updatePoseEstimate();
            distance = distanceToGoal.calculateDistanceToGoal(robotPos,goalPos);


            telemetry.addData("Position", robotPos);
            telemetry.addData("Distance To Goal", distance);


            drivetrain.update();
            intaker.update();
            shooter.update();
            distanceToGoal.update();
            shooter.setDistanceToGoal(distance);


            drivetrain.updateCtrls(gamepad1, gamepad2);
            intaker.updateCtrls(gamepad1, gamepad2);
            shooter.updateCtrls(gamepad1, gamepad2);
            if (!gamepad1.right_bumper && !gamepad1.b && !gamepad1.left_bumper && shooter.getState() == Shooter2.ShooterState.READY_DISTANCE && distance <= 100 && !gamepad1.dpad_right){
                if (!set){
                    commandTime.reset();
                    set = true;
                }
                if (commandTime.seconds() >= 0.2){
                    intaker.Intake();
                }
            } else {
                set = false;

            }

            if (!gamepad1.right_bumper && !gamepad1.b && !gamepad1.left_bumper && shooter.getState() == Shooter2.ShooterState.STOPPING){
                intaker.IntakeIdle();
            }





            if (runtime.seconds() >= 140) {
                isEndgame = true;
            } else {
                isEndgame = false;}


            telemetry.addData("Time Period", isEndgame ? "TeleOp" : "Go Park");

            telemetry.update();


        }
    }


}
