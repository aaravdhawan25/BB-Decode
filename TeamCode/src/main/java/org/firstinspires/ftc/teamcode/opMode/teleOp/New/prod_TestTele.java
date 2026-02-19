package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
@TeleOp(name = "TeleOp Blue Bad", group = "ABC")
public class prod_TestTele extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-40.4,-20, Math.toRadians(245));

    boolean isWaypointing = false;

    Action waypointing;


    Drivetrain drivetrain;
    Intaker intaker;
    Shooter shooter;

    DistanceToGoal distanceToGoal;

    ElapsedTime commandTime = new ElapsedTime();

    boolean isEndgame = false;

    public double xyP = 0.23;

    public double headingP = 0.23;


    public static Vector2d goalPos = new Vector2d(-70,-70);

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

    public boolean isActive = false;

    public Pose2d lockTarget = null;


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
            PoseVelocity2d velocity = follower.updatePoseEstimate();
            double yVel = velocity.linearVel.y;
            double xVel = velocity.linearVel.x;
            double angVel = velocity.angVel;
            runtime.startTime();
            commandTime.startTime();
            FarShoot.startTime();
            robotPos = follower.localizer.getPose().position;
            currPos = follower.localizer.getPose();
            follower.updatePoseEstimate();
            distance = distanceToGoal.calculateDistanceToGoal(robotPos,goalPos);
            turnAngle = distanceToGoal.calculateHeadingToGoal(robotPos, goalPos);

            telemetry.addData("Position", robotPos);
            telemetry.addData("Distance To Goal", distance);
            telemetry.addData("Turn Angle", turnAngle);
            telemetry.addData("X Vel", xVel);
            telemetry.addData("Y Vel", yVel);
            telemetry.addData("Angle Vel", angVel);



            intaker.update();
            shooter.update();
            distanceToGoal.update();
            shooter.setDistanceToGoal(distance);



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
            if (!gamepad1.right_bumper && !gamepad1.b && !gamepad1.left_bumper && shooter.getState() == Shooter.ShooterState.READY_CLOSE && !gamepad1.dpad_right){
                if (!set2){
                    commandTime.reset();
                    set2 = true;
                }
                if (commandTime.seconds() >= 0.2){
                    intaker.Intake();
                }
            } else {
                set2 = false;

            }


//            TrajectoryActionBuilder waypoint = follower.actionBuilder(currPos)
//                    .turnTo(Math.toRadians(turnAngle));
//
//            if (gamepad1.triangleWasPressed()){
//                waypointing = waypoint.build();
//                isWaypointing = true;
//            }
//
////            if (!(gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0)) {
////                isWaypointing = false;
////
////            }
//
//            if (isWaypointing && waypointing != null) {
//                if(!waypointing.run(packet)){
//                    isWaypointing = false;
//                }
//
//            } else {
//                drivetrain.update();
//                drivetrain.updateCtrls(gamepad1, gamepad2);
//            }

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

            FtcDashboard.getInstance().sendTelemetryPacket(packet);






            if (runtime.seconds() >= 140) {
                isEndgame = true;
            } else {
                isEndgame = false;}


            telemetry.addData("Time Period", isEndgame ? "TeleOp" : "Go Park");

            telemetry.update();


        }
    }

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
