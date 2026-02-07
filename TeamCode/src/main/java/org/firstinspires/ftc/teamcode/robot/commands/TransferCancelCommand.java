package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;

public class TransferCancelCommand extends SequentialCommandGroup {
    public TransferCancelCommand(Robot robot){
        addCommands(
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new ShooterCommand(robot, ShooterCMD.ShooterState.STOP),
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
        );
    }
}
