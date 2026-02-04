package org.firstinspires.ftc.teamcode.subsystem.New;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants.BlockerConstants;

public class Blocker implements Subsystem {
    Servo blocker;
    BlockerState state;
    public Blocker(Servo blocker){
        this.blocker = blocker;
    }

    public void setState(BlockerState state){
        this.state = state;
        switch (state){
            case BLOCKED:
                blocker.setPosition(BlockerConstants.BlockerClosed);
                break;
            case UNBLOCKED:
                blocker.setPosition(BlockerConstants.BlockerOpen);
                break;
            default:
                blocker.setPosition(BlockerConstants.BlockerClosed);
                break;
        }
    }


    public BlockerState getState(){
        return state;
    }
    public enum BlockerState{
        BLOCKED,
        UNBLOCKED
    }
}
