
package org.firstinspires.ftc.teamcode.pedroPathing;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;

@Configurable
public class CloseAuto extends OpMode {
    private TelemetryManager panelsTelemetry;

    public FieldManager field;
    public Follower follower;
    private int pathState;
    private CloseAutoPaths paths;

    SequentialCommandGroup auto;

    Robot robot;

    String color;

    public CloseAuto(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        robot = new Robot(hardwareMap, color);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new CloseAutoPaths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
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

        public CloseAutoPaths(Follower follower) {
            ReverseOut = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.048, 122.024),

                                    new Pose(59.277, 84.386)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(136))

                    .build();

            IntakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.277, 84.386),
                                    new Pose(64.849, 56.006),
                                    new Pose(15.446, 59.831)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.446, 59.831),

                                    new Pose(58.747, 84.169)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(136))

                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.747, 84.169),

                                    new Pose(13.301, 64.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(-200))

                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.301, 64.000),
                                    new Pose(23.536, 49.759),
                                    new Pose(11.361, 46.072)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-200), Math.toRadians(-220))

                    .build();

            ReturnGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.361, 46.072),
                                    new Pose(67.163, 62.807),
                                    new Pose(59.036, 83.928)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-220), Math.toRadians(136))

                    .build();

            IntakeSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.036, 83.928),

                                    new Pose(17.145, 83.807)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.145, 83.807),

                                    new Pose(59.048, 83.651)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(136))

                    .build();

            IntakeSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.048, 83.651),
                                    new Pose(55.928, 30.223),
                                    new Pose(12.639, 35.711)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ReturnSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.639, 35.711),
                                    new Pose(44.187, 44.386),
                                    new Pose(63.253, 100.482)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(140))

                    .build();
        }
    }

    private CommandGroupBase shootThree() {
        return new SequentialCommandGroup(
                new TransferCommand(robot),
                new TransferCancelCommand(robot, ShooterCMD.ShooterState.MATH),
                new IntakeCommand(robot, Intake.IntakeState.ON)
        );
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}
    