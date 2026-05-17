package org.firstinspires.ftc.teamcode.subsystem.New;

import static org.firstinspires.ftc.teamcode.subsystem.Drivetrain.lbPower;
import static org.firstinspires.ftc.teamcode.subsystem.Drivetrain.lfDP;
import static org.firstinspires.ftc.teamcode.subsystem.Drivetrain.lfPower;
import static org.firstinspires.ftc.teamcode.subsystem.Drivetrain.rbPower;
import static org.firstinspires.ftc.teamcode.subsystem.Drivetrain.rfPower;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.angleTolerance;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.cameraYaw;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.kP;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.kPOdo;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.maxPower;
import static org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants.poseUpdateIntervalMS;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

public class LLCam implements Subsystem {
    private double lastTy = 0.0;
    private double lastTx = 0.0;
    private Pose3D BotPose = new Pose3D(new Position(DistanceUnit.INCH, 0,0,0, 0), new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    private long lastSeenTimeMs = 0;
    public static boolean isAligning = false;
    
    Telemetry telemetry;

    boolean blue = false;

    public static double rotationalPower;



    public static boolean isAligned = false;
    private static final long TARGET_HOLD_MS = 250;
    private long time, lastRobotPoseUpdateTimeMS = 0;

    public static alignState state;

    @Override
    public void init() {
        setState(alignState.STOP);
        CameraConstants.distanceToGoalLL = 0;
    }

    @Override
    public void update() {
        updateTarget();
        setState(state);
        publishTelem();
        updateDistanceToGoalLL(target);

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.squareWasPressed()){
            state = alignState.MATH_CAMERA;
            
        }
        if (gp1.squareWasReleased()){
            state = alignState.STOP;
        }
    }

    public static class TagTarget {
        public boolean hasTarget = false;
        public Pose3D botPose;
        public static int id = -1;
        public double tX = 0.0;
        public double tY = 0.0;
        public double ambiguity;
    }

    private final Limelight3A limelight;
    public static TagTarget target = new TagTarget();

    public LLCam(Limelight3A limelight, String color, Telemetry telemetry) {
        blue = color.equals("BLUE");
        this.limelight = limelight;
        this.telemetry = telemetry;
        int pipelineIndex = getPipeline();

        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    public void setState(alignState state){
        this.state = state;
        switch (state){
            case MATH_CAMERA:
                isAligning = true;
                TagTarget tag = target;
                if (tag == null || !tag.hasTarget) {

                } else{
                    ShooterCMD.distanceToGoal = updateDistanceToGoalLL(target);
                    pointToGoalCamera(tag);
                }
                break;
            case STOP:
                Drivetrain.lfMotor.setPower(lfPower);
                Drivetrain.lbMotor.setPower(lbPower);
                Drivetrain.rfMotor.setPower(rfPower);
                Drivetrain.rbMotor.setPower(rbPower);
                break;
        }

    }

    public static alignState getState(){
        return state;

    }

    public int getPipeline(){
        int pip = 1;

        if (blue){
            pip = 0;
        }

        if (!blue){
            pip = 1;
        }
        return pip;
    }

    public static double updateDistanceToGoalLL(TagTarget target){
        double totalPitchDeg = target.tY + cameraYaw;
        if (totalPitchDeg == 90 || totalPitchDeg == 270) {
            return 0;
        }
        double tan = Math.tan(Math.toRadians(totalPitchDeg));
        if (tan == 0) {
            return 0;
        }
        return (CameraConstants.goalDY / tan);

    }



    public void pointToGoalCamera(TagTarget tag) {
        if (tag == null || !tag.hasTarget) return;
        double tX = tag.tX;
        double tY = tag.tY;
        double rotationPower = kP * tX;
        rotationalPower = rotationPower;

        rotationPower = Math.max(-maxPower, Math.min(maxPower, rotationPower));
        if (Math.abs(tX) < 0.5){
            return;
        }

        Drivetrain.lfMotor.setPower(-rotationPower);
        Drivetrain.lbMotor.setPower(-rotationPower);
        Drivetrain.rfMotor.setPower(rotationPower);
        Drivetrain.rbMotor.setPower(rotationPower);

        telemetry.addData("x", tX);
        telemetry.addData("y",tY);
        telemetry.addData("power", rotationPower);
        telemetry.addData("CamkP", kP);
        telemetry.addData("OdoKp", kPOdo);
        if (Math.abs(tX) < angleTolerance) {
            isAligned = true;
        } else {
            isAligned = false;
        }

        telemetry.addData("Aligned Properly", isAligned && isAligning);
        telemetry.addData("Math Camera", true);
    }

    public TagTarget getTargetTag() {return target;}
    

    private void updateTarget() {
        LLResult result = limelight.getLatestResult();
        long now = getTime();
        if (result != null && result.isValid()) {
            target.hasTarget = true;
            target.tX = result.getTx();
            target.tY = result.getTy();
            lastTx = target.tX;
            lastTy = target.tY;
            CameraConstants.distanceToGoalLL = updateDistanceToGoalLL(target);
            return;
        }

        if (now - lastSeenTimeMs <= TARGET_HOLD_MS) {
            target.hasTarget = true;
            target.tX = lastTx;
            target.tY = lastTy;
            return;
        }
        target.hasTarget = false;
        target.id = getTarget();
        telemetry.addData("ts works", true);
        CameraConstants.distanceToGoalLL = updateDistanceToGoalLL(target);
    }

    private void publishTelem() {
        telemetry.addData("LL Has Target", target.hasTarget);
        if (!target.hasTarget) return;

        telemetry.addData("LL Tag ID", target.id);
        telemetry.addData("LL Target Distance", CameraConstants.distanceToGoalLL);
    }

    public enum alignState{
        MATH_CAMERA, STOP
    }

    public long getTime(){
        return  time;
    }

    public void updateTime(long time){
        this.time = time;

    }


    public int getTarget(){

        int target = -1;
        if (blue){
            target = 20;
        }

        if (!blue){
            target = 24;
        }

        return target;

    }

}
