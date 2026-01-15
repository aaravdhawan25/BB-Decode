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
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class RedFar extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-49.5, 49.5, Math.toRadians(130));

    MecanumDrive follower;

    Shooter2 shooter;
    Intaker intaker;


    Telemetry telemetry;
    Action preloads, intake1, shoot1, intake2, shoot2, intake3, shoot3, park;
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
        PARK,
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
                .strafeToLinearHeading(new Vector2d(-29.1,29),Math.toRadians(130),new TranslationalVelConstraint(13));
        TrajectoryActionBuilder intakeSpike1 = preloadsShoot.fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(-23.3,24,Math.toRadians(110)), Math.toRadians(345),
                        new TranslationalVelConstraint(30))
                .splineToSplineHeading(new Pose2d(-12,53,Math.toRadians(90)),Math.toRadians(90),
                        new TranslationalVelConstraint(30))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeSpike1.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(-33,22,Math.toRadians(360-230)),Math.toRadians(360-150))
                .waitSeconds(3.5);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToLinearHeading(new Pose2d(5.3,27.8,Math.toRadians(360-280)), Math.toRadians(360-310),
                        new TranslationalVelConstraint(30))
                .splineToSplineHeading(new Pose2d(11.5,53,Math.toRadians(360-270)),Math.toRadians(360-270)

                        ,new TranslationalVelConstraint(30))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.fresh()
                .strafeToLinearHeading(new Vector2d(-33,22.7), Math.toRadians(360-230))
                .waitSeconds(3.5);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.fresh()
                .setTangent(Math.toRadians(360-45))
                .splineToSplineHeading(new Pose2d(29,28.6,Math.toRadians(360-280)), Math.toRadians(360-330),
                        new TranslationalVelConstraint(30)
                )
                .splineToLinearHeading(new Pose2d(36, 53,Math.toRadians(360-270)), Math.toRadians(360-270),
                        new TranslationalVelConstraint(30));
        TrajectoryActionBuilder shootPos3 = intakeSpike3.fresh()
                .strafeToLinearHeading(new Vector2d(-33,23.8), Math.toRadians(360-230))
                .waitSeconds(3.5);
        TrajectoryActionBuilder parkPos = shootPos3.fresh()
                .strafeToLinearHeading(new Vector2d(0,48.8),Math.toRadians(360-180));


        preloads = preloadsShoot.build();
        intake1 = intakeSpike1.build();
        shoot1 = shootPos1.build();
        intake2 = intakeSpike2.build();
        shoot2 = shootPos2.build();
        intake3 = intakeSpike3.build();
        shoot3 = shootPos3.build();
        park = parkPos.build();


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
                    shooter.spinUpClose();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_CLOSE){
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
                    shooter.spinUpClose();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_CLOSE){
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
                    shooter.spinUpClose();
                }

                if (shooter.getState() == Shooter2.ShooterState.READY_CLOSE){
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
                    shooter.spinUpClose();
                }
                if (shooter.getState() == Shooter2.ShooterState.READY_CLOSE){
                    intaker.Intake();
                }

                if (!currentAction) {
                    state = AutoStates.PARK;
                    time.reset();
                }
                break;

            case PARK:
                currentAction = park.run(packet);
                shooter.stop();
                intaker.IntakeIdle();

                if (!currentAction){
                    state = AutoStates.END;
                    time.reset();
                }

            case END:
                break;

        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}