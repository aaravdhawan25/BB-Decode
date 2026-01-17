package org.firstinspires.ftc.teamcode.opMode.auto.New;

import android.icu.lang.UScript;

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
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;

@Config
@Autonomous(name = "Red Side Close Auto", group = "Autonomous")
public class RedFar extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-49.9, 49.7, Math.toRadians(360-230));

    MecanumDrive follower;

    Shooter2 shooter;
    Intaker intaker;


    Telemetry telemetry;
    Action preloads, intake1, shoot1, intake2, shoot2, intake3, shoot3;
    boolean currentAction;
    enum AutoStates {
        START,
        PRELOADS,
        INTAKE1,
        SHOOT1,
        INTAKE2,
        SHOOT2,
        INTAKE3,
        SHOOT3,
        END
    }

    AutoStates state = AutoStates.START;


    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new MecanumDrive(hardwareMap, initialPose);
        shooter = new Shooter2(hardwareMap, telemetry);
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
            shooter.update();
            intaker.update();


            update();



            telemetry.addData("State", state);
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
                .strafeToLinearHeading(new Vector2d(-29.1,29),Math.toRadians(360-230),new TranslationalVelConstraint(13))
                .waitSeconds(1);
        TrajectoryActionBuilder intakeSpike1 = preloadsShoot.fresh()
                .setTangent(Math.toRadians(360-70))
                .splineToLinearHeading(new Pose2d(-23.3,24,Math.toRadians(360-250)), Math.toRadians(360-15),
                        new TranslationalVelConstraint(40))

                .splineToSplineHeading(new Pose2d(-12,55,Math.toRadians(360-270)),Math.toRadians(360-270),
                        new TranslationalVelConstraint(40))
                //.splineToLinearHeading(new Pose2d(-14.5,-49.3,Math.toRadians(270)), Math.toRadians(200))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeSpike1.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(-29,22,Math.toRadians(360-230)),Math.toRadians(360-150))
//                .splineTo(new Vector2d(-21.9,-22),Math.toRadians(130))
                .waitSeconds(3);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToLinearHeading(new Pose2d(6,24.8,Math.toRadians(360-270)), Math.toRadians(360-310),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(11.5,56.5,Math.toRadians(360-270)),Math.toRadians(360-270)

                        ,new TranslationalVelConstraint(40))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.fresh()
                .strafeToLinearHeading(new Vector2d(-28.5,22.7), Math.toRadians(360-230))
                .waitSeconds(3.5);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(26.5 ,25,Math.toRadians(360-280)), Math.toRadians(360-330),
                        new TranslationalVelConstraint(40)
                )
                .splineToLinearHeading(new Pose2d(36, 58,Math.toRadians(360-270)), Math.toRadians(360-270),
                        new TranslationalVelConstraint(40));
        TrajectoryActionBuilder shootPos3 = intakeSpike3.fresh()
                .strafeToLinearHeading(new Vector2d(-40.4,21), Math.toRadians(360-242))
                .waitSeconds(3.5);
        preloads = preloadsShoot.build();
        intake1 = intakeSpike1.build();
        shoot1 = shootPos1.build();
        intake2 = intakeSpike2.build();
        shoot2 = shootPos2.build();
        intake3 = intakeSpike3.build();
        shoot3 = shootPos3.build();


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
                if (time.seconds() <= 1){
                    shooter.spinUpAuto();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_AUTO){
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
                    shooter.spinUpAuto();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_AUTO){
                    intaker.Intake();
                }

                if (!currentAction) {
                    state = AutoStates.INTAKE2;
                    time.reset();

                }
                break;
            case INTAKE2:
                currentAction = intake2.run(packet);
                shooter.stop();
                intaker.Intake();

                if (!currentAction) {
                    state = AutoStates.SHOOT2;
                    time.reset();
                }
                break;

            case SHOOT2:
                currentAction = shoot2.run(packet);
                intaker.IntakeIdle();
                if (time.seconds() >= 1.84 && time.seconds() <= 2.1){
                    shooter.spinUpAuto();
                }

                if (shooter.getState() == Shooter2.ShooterState.READY_AUTO){
                    intaker.Intake();
                }

                if (!currentAction) {
                    state = AutoStates.INTAKE3;
                    time.reset();
                }
                break;


            case INTAKE3:
                currentAction = intake3.run(packet);
                shooter.stop();
                intaker.Intake();

                if (!currentAction){
                    state = AutoStates.SHOOT3;
                    time.reset();
                }
                break;


            case SHOOT3:
                currentAction = shoot3.run(packet);
                intaker.IntakeIdle();
                if (time.seconds() >= 2.12 && time.seconds() <= 2.4){
                    shooter.spinUpAuto();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_AUTO){
                    intaker.Intake();
                }

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