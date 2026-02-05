package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;

public class ShooterCommand extends SequentialCommandGroup {
    public ShooterCommand(Robot robot, ShooterCMD.ShooterState state){
        addCommands(
                new InstantCommand(() -> robot.shooter.setState(state))
        );
    }
}
