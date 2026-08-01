package org.firstinspires.ftc.teamcode.opMode.auto.New;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opMode.auto.CloseAutoPoseData;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants;

public class Pedro18Ball extends OpMode {

    private ElapsedTime timer;
    private ElapsedTime stateTimer;
    private DashboardPoseTracker dashboardPoseTracker;

    private Follower follower;
    private Shooter shooter;
    private Intaker intaker;

    private CloseAutoPaths paths;
    private String color;

    public enum AutoState {
        REVERSE_OUT,
        INTAKE_SPIKE_2,
        RETURN_SPIKE_2,
        GO_TO_GATE,
        GATE_INTAKE,
        WAIT_GATE_INTAKE,
        RETURN_GATE,

        GO_TO_GATE2,
        GATE_INTAKE2,
        WAIT_GATE_INTAKE2,
        RETURN_GATE2,
        INTAKE_SPIKE_1,
        WAIT_INTAKE_1,
        RETURN_SPIKE_1,
        INTAKE_SPIKE_3,
        WAIT_INTAKE_3,
        RETURN_SPIKE_3,
        SHOOTING_SPINUP,
        SHOOTING_TRANSFER,
        DONE
    }

    private AutoState currentState;
    private AutoState nextStateAfterShoot;

    public Pedro18Ball(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        stateTimer = new ElapsedTime();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        paths = new CloseAutoPaths(follower, color);
        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);

        shooter = new Shooter(hardwareMap, telemetry);
        shooter.init();

        intaker = new Intaker(hardwareMap, telemetry);
        intaker.init();
        intaker.IntakeIdle();
        Drawing.drawRobot(CloseAutoPoseData.mirror(CloseAutoPoseData.START_POSE, color), "#4CAF50");
        Drawing.sendPacket();
        dashboardPoseTracker.update();
    }

    @Override
    public void start() {
        timer.reset();
        timer.startTime();
        stateTimer.reset();
        stateTimer.startTime();

        follower.update();

        setPathState(AutoState.REVERSE_OUT);

        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        dashboardPoseTracker.update();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        intaker.update();
        dashboardPoseTracker.update();

        switch (currentState) {
            case REVERSE_OUT:
                if (!follower.isBusy()) {
                    prepareShoot(AutoState.INTAKE_SPIKE_2);
                }
                break;

            case INTAKE_SPIKE_2:
                if (!follower.isBusy()) {
                    setPathState(AutoState.RETURN_SPIKE_2);
                }
                break;

            case RETURN_SPIKE_2:
                if (follower.isBusy()){
                    intaker.IntakeIdle();
                }
                if (!follower.isBusy()) {
                    intaker.IntakeIdle();
                    prepareShoot(AutoState.INTAKE_SPIKE_1);
                }
                break;

            case GO_TO_GATE:
                if (!follower.isBusy()) {
                    setPathState(AutoState.GATE_INTAKE);
                }
                break;

            case GATE_INTAKE:
                if (!follower.isBusy()) {
                    setPathState(AutoState.WAIT_GATE_INTAKE);
                }
                break;

            case WAIT_GATE_INTAKE:
                if (stateTimer.seconds() > 0.6) {
                    setPathState(AutoState.RETURN_GATE);
                }
                break;

            case RETURN_GATE:
                if (!follower.isBusy()) {
                    prepareShoot(AutoState.INTAKE_SPIKE_3);
                }
                break;

            case INTAKE_SPIKE_1:
                if (!follower.isBusy()) {
                    setPathState(AutoState.RETURN_SPIKE_1);
                }
                break;

            case RETURN_SPIKE_1:
                if (follower.isBusy()){
                    intaker.IntakeIdle();
                }
                if (!follower.isBusy()) {
                    prepareShoot(AutoState.GO_TO_GATE);
                }
                break;

            case INTAKE_SPIKE_3:
                if (!follower.isBusy()) {
                    setPathState(AutoState.RETURN_SPIKE_3);
                }
                break;

            case RETURN_SPIKE_3:
                if (follower.isBusy()){
                    intaker.IntakeIdle();
                }
                if (!follower.isBusy()) {
                    prepareShoot(AutoState.DONE);
                }
                break;

            case SHOOTING_SPINUP:
                if (shooter.getState() == Shooter.ShooterState.READY_CLOSE) {
                    stateTimer.reset();
                    currentState = AutoState.SHOOTING_TRANSFER;
                }
                break;

            case SHOOTING_TRANSFER:
                if (stateTimer.milliseconds() < 100) {

                } else if (stateTimer.milliseconds() < 800) {
                    intaker.TransferShootSub();
                } else {
                    intaker.IntakeIdle();
                    shooter.stop();
                    setPathState(nextStateAfterShoot);
                }
                break;

            case DONE:
                shooter.stop();
                intaker.IntakeIdle();
                break;
        }

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        telemetry.addData("POSE", follower.getPose());
        telemetry.addData("STATE", currentState.toString());
        telemetry.addData("TIMER", timer.seconds());
        telemetry.update();
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();

        switch (newState) {
            case REVERSE_OUT:
                intaker.IntakeIdle();
                follower.followPath(paths.ReverseOut);
                break;
            case INTAKE_SPIKE_2:
                intaker.Intake();
                follower.followPath(paths.IntakeSpike2);
                break;
            case RETURN_SPIKE_2:
                follower.followPath(paths.ReturnSpike2);
                break;
            case GO_TO_GATE:
                follower.followPath(paths.Gate);
                break;
            case GATE_INTAKE:
                intaker.Intake();
                follower.followPath(paths.GateIntake);
                break;
            case RETURN_GATE:
                intaker.IntakeIdle();
                follower.followPath(paths.ReturnGate);
                break;
            case INTAKE_SPIKE_1:
                intaker.Intake();
                follower.followPath(paths.IntakeSpike1);
                break;
            case RETURN_SPIKE_1:
                follower.followPath(paths.ReturnSpike1);
                break;
            case INTAKE_SPIKE_3:
                intaker.Intake();
                follower.followPath(paths.IntakeSpike3);
                break;
            case RETURN_SPIKE_3:
                follower.followPath(paths.ReturnSpike3);
                break;
            default:
                break;
        }
    }

    private void prepareShoot(AutoState nextState) {
        nextStateAfterShoot = nextState;
        intaker.IntakeIdle();
        shooter.spinUpClose();
        currentState = AutoState.SHOOTING_SPINUP;
    }

    public static class CloseAutoPaths {
        public PathChain ReverseOut, IntakeSpike2, ReturnSpike2, Gate, GateIntake;
        public PathChain ReturnGate, IntakeSpike1, ReturnSpike1, IntakeSpike3, ReturnSpike3;

        public CloseAutoPaths(Follower follower, String color) {

            Pose startPose = CloseAutoPoseData.mirror(CloseAutoPoseData.START_POSE, color);
            Pose shootingPose = CloseAutoPoseData.mirror(CloseAutoPoseData.SHOOTING_POSE, color);
            Pose finalShootingPose = CloseAutoPoseData.mirror(CloseAutoPoseData.FINAL_SHOOT, color);
            Pose firstIntake = CloseAutoPoseData.mirror(CloseAutoPoseData.INTAKE1, color);
            Pose mid2Curve = CloseAutoPoseData.mirror(CloseAutoPoseData.MID2_CURVE, color);
            Pose secondIntake = CloseAutoPoseData.mirror(CloseAutoPoseData.SECOND_INTAKE, color);
            Pose leverRetCont = CloseAutoPoseData.mirror(CloseAutoPoseData.LEVER_RETURN_CONTROL, color);
            Pose finalIntake = CloseAutoPoseData.mirror(CloseAutoPoseData.FINAL_INTAKE, color);
            Pose leverPose = CloseAutoPoseData.mirror(CloseAutoPoseData.LEVER, color);
            Pose intake3Cont = CloseAutoPoseData.mirror(CloseAutoPoseData.INTAKE3Cont, color);
            Pose finalShootCont = CloseAutoPoseData.mirror(CloseAutoPoseData.finalShootCont, color);
            Pose leverIntakePose = CloseAutoPoseData.mirror(CloseAutoPoseData.LEVER_INTAKE, color);
            Pose leverCont = CloseAutoPoseData.mirror(CloseAutoPoseData.LEVER_CONTROL, color);

            double heading180 = CloseAutoPoseData.mirrorHeading(AutoConstants.Heading180, color);
            double leverHitHeading = CloseAutoPoseData.mirrorHeading(AutoConstants.leverHitHeading, color);
            double leverHeading = CloseAutoPoseData.mirrorHeading(AutoConstants.leverHeading, color);
            double finalShootHeading = CloseAutoPoseData.mirrorHeading(AutoConstants.finalShootHeading, color);
            double whileMovingStartHeading = CloseAutoPoseData.mirrorHeading(AutoConstants.startShootMoveHeading, color);
            double shootHeading = CloseAutoPoseData.mirrorHeading(AutoConstants.endShootMoveHeading, color);

            follower.setStartingPose(startPose);

            ReverseOut = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(whileMovingStartHeading), Math.toRadians(shootHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            IntakeSpike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, mid2Curve, secondIntake))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            ReturnSpike2 = follower.pathBuilder()
                    .addPath(new BezierLine(secondIntake, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            Gate = follower.pathBuilder()
                    .addPath(new BezierLine(shootingPose, leverPose))
                    .setLinearHeadingInterpolation(Math.toRadians(shootHeading), Math.toRadians(leverHitHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(leverPose, leverCont, leverIntakePose))
                    .setLinearHeadingInterpolation(Math.toRadians(leverHitHeading), Math.toRadians(leverHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)

                    .build();

            ReturnGate = follower.pathBuilder()
                    .addPath(new BezierCurve(leverIntakePose, leverRetCont, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(leverHeading), Math.toRadians(shootHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplierGate)
                    .build();

            IntakeSpike1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootingPose, firstIntake))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            ReturnSpike1 = follower.pathBuilder()
                    .addPath(new BezierLine(firstIntake, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            IntakeSpike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootingPose, intake3Cont, finalIntake))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();

            ReturnSpike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(finalIntake, finalShootCont, finalShootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(finalShootHeading))
                    .setZeroPowerAccelerationMultiplier(AutoConstants.decelMultiplier)
                    .build();


        }
    }
}