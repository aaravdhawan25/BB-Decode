package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot, ShooterCMD.ShooterState shooterState) {
        addCommands(
                new ShooterCommand(robot, shooterState),
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> robot.shooter.atTargetSpeed()),
                        new WaitCommand(1000)
                ),
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                new WaitCommand(100),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(750)
        );

    }

    public TransferCommand(Robot robot){
        this(robot, robot.shooter.getState());
    }
}
