package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(Robot robot, Intake.IntakeState state){
        super(() -> robot.intake.setState(state));
    }
}
