package org.firstinspires.ftc.teamcode.robot.commands;

import android.graphics.Paint;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.LLCam;

public class AlignCommand extends SequentialCommandGroup {
    public AlignCommand(Robot robot, LLCam.alignState state){
        addCommands(
                new InstantCommand(()-> robot.limelightCamera.setState(state))
        );
    }
}
