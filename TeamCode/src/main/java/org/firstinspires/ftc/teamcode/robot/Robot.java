package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.subsystem.New.TurretCMD;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;
@Config
public class Robot {

    DcMotorEx shooterMotor, shooterMotor2, intakeMotor, transferMotor;
    public ShooterCMD shooter;
    public Intake intake;
    public Blocker blocker;

    public TurretCMD turret;

    public static double compensatedDistance = 0;

    public static PoseVelocity2d robotVel;

    public Pose2d lockTarget = null;

    public static double xyP = 0.23;
    public static double headingP = 0.23;

    public Servo blockerServo, turretServo1, turretServo2, hoodServo;

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
        shooterMotor2 =  hardwareMap.get(DcMotorEx.class, "shooter2");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        turretServo1 = hardwareMap.get(Servo.class, "turretLeft");
        turretServo2 = hardwareMap.get(Servo.class, "turretRight");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooter = new ShooterCMD(shooterMotor,shooterMotor2, hoodServo);
        blocker = new Blocker(blockerServo);
        turret = new TurretCMD(turretServo1, turretServo2);
        intake = new Intake(intakeMotor,transferMotor);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServo1.setDirection(Servo.Direction.REVERSE);
        turretServo2.setDirection(Servo.Direction.REVERSE);
        follower = new MecanumDrive(hardwareMap, getStartPose());
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(intake,shooter,blocker, turret);

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

    public Pose2d getCurrPos(){
        return currPos;

    }

    public void setLockTarget(){
        lockTarget = getCurrPos();
    }

    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = follower.localizer.getPose();

        Vector2d difference = targetPos.position.minus(currPos.position);

        // Manually rotate the difference vector by -currPos.heading
        double angle = -currPos.heading.toDouble();
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        Vector2d xy = new Vector2d(
                difference.x * cos - difference.y * sin,
                difference.x * sin + difference.y * cos
        );

        double heading = targetPos.heading.toDouble() - currPos.heading.toDouble();
        // Normalize heading to [-PI, PI]
        heading = Rotation2d.exp(heading).toDouble();

        follower.setDrivePowers(new PoseVelocity2d(
                xy.times(xyP),
                heading * headingP
        ));
    }

    public double calculateHeadingToGoal(Vector2d robotPos, Vector2d goalPos) {
        Vector2d toGoal = goalPos.minus(robotPos);
        double angleToGoalDeg = Math.toDegrees(Math.atan2(toGoal.y, toGoal.x));
        return angleToGoalDeg;
    }

    public static void updateCompensatedDistance() {
        if (robotVel == null) return;

        Vector2d toGoal = getGoalPos().minus(robotPos);
        double x = Math.max(toGoal.norm() - ShooterConstants.PASS_THROUGH_POINT_RADIUS, 1.0);
        double y = ShooterConstants.TARGET_Y;
        double a = ShooterConstants.IMPACT_ANGLE_THETA;
        double g = ShooterConstants.G;

        // initial flywheel speed assuming stationary
        double hoodAngle = Math.atan(2 * y / x - Math.tan(a));
        double flywheelSpeed = Math.sqrt(
                g * x * x /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y))
        );

        // get velocity components
        double velMagnitude = Math.hypot(robotVel.linearVel.x, robotVel.linearVel.y);

        // if barely moving just use real distance, no compensation needed
        if (velMagnitude < 0.5) {
            compensatedDistance = x;
            return;
        }

        double fieldAngle = Math.atan2(toGoal.y, toGoal.x);
        double velocityTheta = Math.atan2(robotVel.linearVel.y, robotVel.linearVel.x);

        // decompose velocity into parallel and perpendicular to goal direction
        double coordinateTheta = velocityTheta - fieldAngle;
        double parallelComponent = -Math.cos(coordinateTheta) * velMagnitude;
        double perpendicularComponent = Math.sin(coordinateTheta) * velMagnitude;

        // time of flight using stationary flywheel speed
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));

        // ivr = required radial speed accounting for robot moving toward/away
        double ivr = x / time + parallelComponent;

        // nvr = total horizontal speed needed combining radial and sideways
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);

        // ndr = virtual distance the ball needs to travel
        double ndr = nvr * time;

        compensatedDistance = ndr;
    }

    public void update(){
        CommandScheduler.getInstance().run();
        follower.updatePoseEstimate();
        robotPos = follower.localizer.getPose().position;
        updateCompensatedDistance();
        robotVel = follower.updatePoseEstimate();
        currPos = follower.localizer.getPose();
        shooter.setDistanceToGoal(getDistanceFromGoal());
        calculateHeadingToGoal(robotPos, goalPos);
        shooter.periodic();

        PerTelem.addData("Distance To Goal", getDistanceFromGoal());
        PerTelem.addData("Shooter State", shooter.getState());
        PerTelem.addData("Intake State", intake.getState());
        PerTelem.addData("Blocker State", blocker.getState());
        PerTelem.addData("Cur Pose", follower.localizer.getPose());
        PerTelem.update();
    }

    public void stop(){
        CommandScheduler.getInstance().reset();
    }


}
