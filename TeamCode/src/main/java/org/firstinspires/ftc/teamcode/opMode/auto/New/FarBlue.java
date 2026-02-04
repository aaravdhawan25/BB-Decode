package org.firstinspires.ftc.teamcode.opMode.auto.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.New.DistanceToGoal;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;

@Config
@Autonomous(name = "Blue Side Far Auto", group = "BClass")
public class FarBlue extends LinearOpMode {
    Pose2d initialPose = new Pose2d(58.5, -10, Math.toRadians(215));

    MecanumDrive follower;

    Shooter shooter;
    Intaker intaker;

    DistanceToGoal distanceToGoal;

    Vector2d robotPos = new Vector2d(0,0);
    Vector2d goalPos = new Vector2d(-70,70);

    public double distance = 0;


    Telemetry telemetry;
    Action preloads, intake1, shoot1,leaveSpot;
    boolean currentAction;
    enum AutoStates {
        START,
        PRELOADS,
        INTAKE1,
        SHOOT1,
        LEAVE,
        END
    }

    AutoStates state = AutoStates.START;


    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new MecanumDrive(hardwareMap, initialPose);
        distanceToGoal = new DistanceToGoal(telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        intaker = new Intaker(hardwareMap, telemetry);

        build_paths();

        while (opModeInInit()){
            shooter.init();
            intaker.init();

        }




        waitForStart();
        if (isStopRequested()) return;
        time.startTime();
        time.reset();

        while (opModeIsActive()) {
            follower.updatePoseEstimate();
            Pose2d currentPose = follower.localizer.getPose();
            robotPos = follower.localizer.getPose().position;
            distance = distanceToGoal.calculateDistanceToGoal(robotPos,goalPos);
            shooter.setDistanceToGoal(distance);
            shooter.update();
            intaker.update();


            update();



            telemetry.addData("X", currentPose.position.x);
            telemetry.addData("Y", currentPose.position.y);
            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Current State", state.name());
            telemetry.addData("State Time", "%.2f sec", time.seconds());
            telemetry.update();
        }


    }

    public void build_paths() {
        TrajectoryActionBuilder preloadsShoot = follower.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(58.5, -10), Math.toRadians(215))
                .waitSeconds(5);
        TrajectoryActionBuilder intakeRun1 = preloadsShoot.fresh()
                .strafeToLinearHeading(new Vector2d(61,-60),Math.toRadians(270))
                .setTangent(Math.toRadians(120))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeRun1.fresh()
                .splineToLinearHeading(new Pose2d(58.5,-10,Math.toRadians(215)),Math.toRadians(60))
                .waitSeconds(12);
        TrajectoryActionBuilder leave = shootPos1.fresh()
                .strafeTo(new Vector2d(40,-16));
        preloads = preloadsShoot.build();
        intake1 = intakeRun1.build();
        shoot1 = shootPos1.build();
        leaveSpot = leave.build();



    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();


        switch (state) {
            case START:
                time.reset();
                state = AutoStates.PRELOADS;
            case PRELOADS:
                currentAction = preloads.run(packet);
                intaker.IntakeIdle();
                shooter.spinUpFar();

                if (shooter.getState() == Shooter.ShooterState.READY_FAR){
                    intaker.Intake();
                }

                if (!currentAction) {
                    state = AutoStates.INTAKE1;
                    time.reset();

                }
                break;

            case INTAKE1:
                currentAction = intake1.run(packet);
                shooter.stop();
                intaker.Intake();

                if (!currentAction) {
                    state = AutoStates.SHOOT1;
                    time.reset();
                }
                break;
            case SHOOT1:
                currentAction = shoot1.run(packet);
                intaker.IntakeIdle();
                if (time.seconds() >= 1.4 && time.seconds()<=2){
                    shooter.spinUpFar();
                }
                if (shooter.getState() == Shooter.ShooterState.READY_FAR){
                    intaker.Intake();
                }

                if(time.seconds()>=4&& time.seconds()<=4.2){
                    shooter.stop();
                    intaker.IntakeIdle();
                }

                if (!currentAction) {
                    state = AutoStates.LEAVE;
                    time.reset();

                }
                break;
            case LEAVE:
                currentAction = leaveSpot.run(packet);

                if (!currentAction) {
                    state = AutoStates.END;
                    time.reset();
                }
                break;

            case END:
                break;

        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}