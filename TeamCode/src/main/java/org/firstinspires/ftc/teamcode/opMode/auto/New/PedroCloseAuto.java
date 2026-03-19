package org.firstinspires.ftc.teamcode.opMode.auto.New;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.utils.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

public class PedroCloseAuto extends OpMode {

    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private CloseAutoPaths paths;
    private SequentialCommandGroup auto;
    private String color;

    public PedroCloseAuto(String color) {
        this.color = color;
    }
    @Override
    public void init() {
        timer = new ElapsedTime();
        PerTelem.init(telemetry);
        robot = new Robot(hardwareMap, color);
        paths = new CloseAutoPaths(robot.follower, color);

        CommandScheduler.getInstance().schedule(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
        );

    }

    @Override
    public void loop() {
        PerTelem.addData("POSE", robot.follower.getPose());
        PerTelem.addData("TIMER", timer.seconds());
        robot.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void start(){
        timer.reset();
        CommandScheduler.getInstance().schedule(
                auto
        );

        auto = new SequentialCommandGroup(
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new FollowPathCommand(robot.follower, paths.ReverseOut),
                shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.IntakeSpike2),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.ReturnSpike2),
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                shootThree(),
                new FollowPathCommand(robot.follower, paths.Gate),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.GateIntake),
                new WaitCommand(700),
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.ReturnGate),
                        new IntakeCommand(robot, Intake.IntakeState.OFF)),
                shootThree(),
                new FollowPathCommand(robot.follower, paths.IntakeSpike1),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(100),
                new FollowPathCommand(robot.follower, paths.ReturnSpike1),
                shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.IntakeSpike3),
                new WaitCommand(100),
                new FollowPathCommand(robot.follower, paths.ReturnSpike3),
                shootThree(),
                new ShooterCommand(robot, ShooterCMD.ShooterState.STOP)
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

    }

    private CommandGroupBase shootThree() {
        return new SequentialCommandGroup(
                new TransferCommand(robot, ShooterCMD.ShooterState.MATH),
                new TransferCancelCommand(robot, ShooterCMD.ShooterState.STOP)
        );
    }


    public static class CloseAutoPaths {
        public PathChain ReverseOut;
        public PathChain IntakeSpike2;
        public PathChain ReturnSpike2;
        public PathChain Gate;
        public PathChain GateIntake;
        public PathChain ReturnGate;
        public PathChain IntakeSpike1;
        public PathChain ReturnSpike1;
        public PathChain IntakeSpike3;
        public PathChain ReturnSpike3;

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
            Pose leverPose2 = CloseAutoPoseData.mirror(CloseAutoPoseData.LEVER_SECOND, color);
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
            
            ReverseOut = follower.pathBuilder().
                    addPath(new BezierLine(startPose, shootingPose))
                    .setLinearHeadingInterpolation(Math.toRadians(whileMovingStartHeading), Math.toRadians(shootHeading))
                    .build();

            IntakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootingPose,
                                    mid2Curve,
                                    secondIntake
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    secondIntake,
                                    shootingPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootHeading))

                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierLine(shootingPose, leverPose)
                    ).setLinearHeadingInterpolation(Math.toRadians(shootHeading), Math.toRadians(leverHitHeading))

                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    leverPose,
                                    leverCont,
                                    leverIntakePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(leverHitHeading), Math.toRadians(leverHeading))

                    .build();

            ReturnGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    leverIntakePose,
                                    leverRetCont,
                                    shootingPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(leverHeading), Math.toRadians(shootHeading))

                    .build();

            IntakeSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingPose,
                                    firstIntake
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    firstIntake,
                                    shootingPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(shootHeading))

                    .build();

            IntakeSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootingPose,
                                    intake3Cont,
                                    finalIntake
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    finalIntake,
                                    finalShootCont,
                                    finalShootingPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(finalShootHeading))

                    .build();
        }
    }

}
