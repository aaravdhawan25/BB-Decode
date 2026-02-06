package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.DistanceToGoal;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;

@Config
@TeleOp(name = "TeleOp Red Bad", group = "ABC")
public class prod_TestTele_Red extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-44,19, Math.toRadians(360-246));

    boolean isWaypointing = false;



    Drivetrain drivetrain;
    Intaker intaker;
    Shooter shooter;

    DistanceToGoal distanceToGoal;

    ElapsedTime commandTime = new ElapsedTime();

    boolean isEndgame = false;


    public static Vector2d goalPos = new Vector2d(-70,70);

    Pose2d currPos = new Pose2d(0,0,Math.toRadians(0));

    Vector2d robotPos = new Vector2d(0,0);


    TelemetryPacket packet;
    double robotHeading = 0;
    public static double FarShootDelay1 = 0.6;

    double turnAngle;

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
        packet = new TelemetryPacket();
        drivetrain = new Drivetrain(hardwareMap,telemetry);
        intaker = new Intaker(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
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
            currPos = follower.localizer.getPose();
            follower.updatePoseEstimate();
            distance = distanceToGoal.calculateDistanceToGoal(robotPos,goalPos);
            turnAngle = distanceToGoal.calculateRobotAngle(robotPos, Math.toDegrees(currPos.heading.toDouble()), goalPos);

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
            if (!gamepad1.right_bumper && !gamepad1.b && !gamepad1.left_bumper && shooter.getState() == Shooter.ShooterState.READY_DISTANCE && distance <= 100 && !gamepad1.dpad_right){
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

            if (!gamepad1.right_bumper && !gamepad1.b && !gamepad1.left_bumper && shooter.getState() == Shooter.ShooterState.STOPPING){
                intaker.IntakeIdle();
            }


//            TrajectoryActionBuilder waypoint = follower.actionBuilder(currPos)
//                    .turnTo(Math.toRadians(turnAngle));
//
//            if (gamepad1.triangleWasPressed()){
//                waypointing = waypoint.build();
//                isWaypointing = true;
//            }
//
//            if (!(gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0)) {
//                isWaypointing = false;
//
//            }
//
//            if (isWaypointing && waypointing != null) {
//                if(!waypointing.run(packet)){
//                    isWaypointing = false;
//                }
//
//            }

//            FtcDashboard.getInstance().sendTelemetryPacket(packet);






            if (runtime.seconds() >= 140) {
                isEndgame = true;
            } else {
                isEndgame = false;}


            telemetry.addData("Time Period", isEndgame ? "TeleOp" : "Go Park");

            telemetry.update();


        }
    }


}
