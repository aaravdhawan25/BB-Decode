package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private boolean mirrored = false;
    private double maxPower = 1;
    public FollowPathCommand(Follower follower, PathChain path) {
        this.follower = follower;
        follower.setMaxPower(maxPower);
        this.path = path;
    }

    public FollowPathCommand(Follower follower, PathChain path, int maxPower){
        this.follower = follower;
        follower.setMaxPower(maxPower);
        this.maxPower = maxPower;
        this.path = path;
        ;}
    public FollowPathCommand(Follower follower, Path path) {
        this(follower, new PathChain(path));
    }

    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    public FollowPathCommand setMirror(boolean mirrored){
        this.mirrored = mirrored;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
