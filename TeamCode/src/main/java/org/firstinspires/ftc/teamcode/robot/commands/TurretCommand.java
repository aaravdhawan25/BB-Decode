package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.TurretCMD;
import org.firstinspires.ftc.teamcode.utils.Constants.TurretConstants;

public class TurretCommand extends SequentialCommandGroup {
    public TurretCommand(Robot robot, TurretCMD.TurretState state){
        addCommands(
//                new InstantCommand(()-> robot.turret.setState(state))
        );
    }
}
