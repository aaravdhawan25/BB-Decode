package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

public class Robot {

    DcMotorEx shooterMotor, counterRoller, intakeMotor, transferMotor;
    public ShooterCMD shooter;
    public Intake intake;

    public Blocker blocker;

    public Servo blockerServo;

    public MecanumDrive follower;

    public static boolean blue;

    public static Vector2d goalPos = new Vector2d(-70,-70);

    public static Pose2d START_POSE = new Pose2d(-40.4,-20, Math.toRadians(245));

    public static Pose2d currPos = new Pose2d(0,0,Math.toRadians(0));

    public static Vector2d robotPos = new Vector2d(0,0);



    public Robot(HardwareMap hardwareMap, String color){
        this(hardwareMap);
        blue = color.equals("BLUE");
    }

    public Robot(HardwareMap hardwareMap ){
        shooterMotor =  hardwareMap.get(DcMotorEx.class, "shooter");
        counterRoller =  hardwareMap.get(DcMotorEx.class, "CR");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = new ShooterCMD(shooterMotor,counterRoller);
        blocker = new Blocker(blockerServo);
        intake = new Intake(intakeMotor,transferMotor);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower = new MecanumDrive(hardwareMap, getStartPose());
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(intake,shooter,blocker);

    }

    public static double getDistanceFromGoal(Vector2d robotPos){
        Vector2d goalPos = Robot.getGoalPos();
        Vector2d toGoal = goalPos.minus(robotPos);
        return Math.sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);

    }

    public static double getDistanceFromGoal(){
        return getDistanceFromGoal(Robot.robotPos);
    }

    public static Vector2d getGoalPos(){
        if (blue){
            goalPos = new Vector2d(-70,-70);
        }
        else{
            goalPos = new Vector2d(-70,70);
        }
        return goalPos;
    }

    public static Pose2d getStartPose(){
        if (blue){
            START_POSE = new Pose2d(-40.4,-20, Math.toRadians(245));
        }
        else{
            START_POSE = new Pose2d(-44,19, Math.toRadians(360-246));
        }
        return START_POSE;
    }

    public void update(){
        CommandScheduler.getInstance().run();
        follower.updatePoseEstimate();
        robotPos = follower.localizer.getPose().position;
        shooter.setDistanceToGoal(getDistanceFromGoal());
        shooter.periodic();

        PerTelem.addData("Distance To Goal", getDistanceFromGoal());
        PerTelem.addData("Shooter State", shooter.getState());
        PerTelem.addData("Intake State", intake.getState());
        PerTelem.addData("Blocker State", blocker.getState());
        PerTelem.update();
    }

    public void stop(){
        CommandScheduler.getInstance().reset();
    }


}
