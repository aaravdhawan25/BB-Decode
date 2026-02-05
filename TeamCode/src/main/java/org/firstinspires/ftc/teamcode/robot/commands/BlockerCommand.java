package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;

public class BlockerCommand extends SequentialCommandGroup {
    public BlockerCommand(Robot robot, Blocker.BlockerState state){
        addCommands(
                new InstantCommand(() -> robot.blocker.setState(state))
        );
    }
}
