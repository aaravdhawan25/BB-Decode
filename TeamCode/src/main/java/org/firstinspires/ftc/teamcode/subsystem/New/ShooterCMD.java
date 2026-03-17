package org.firstinspires.ftc.teamcode.subsystem.New;

import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MID;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.IDLE_SHOOTER;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.TESTING_HOOD_POS;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.crTargetRPM;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.targetRPM;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;
import org.opencv.core.Mat;

@Config
public class ShooterCMD implements Subsystem {
    Telemetry telemetry;

    public DcMotorEx shooterMotor,shooterMotor2;

    Servo hoodServo;

    double HOOD_SERVO_POS;
    public static double distanceToGoal = 0;
    public PIDController pidController;

    public ShooterState state = ShooterState.MATH;

    public static double error = 100;

    public long lastTime = 0;

    public ShooterCMD(DcMotorEx shooterMotor, DcMotorEx shooterMotor2, Servo hoodServo){
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
        this.hoodServo = hoodServo;
    }
    
    public void setState(ShooterState state){
        this.state = state;
        switch (state){
            case CLOSE:
                targetRPM = ShooterConstants.CLOSE_SHOOTER_RPM;
                HOOD_SERVO_POS = HOOD_SERVO_MAX;
                break;
            case FAR:
                targetRPM = ShooterConstants.FAR_SHOOTER_RPM;
                HOOD_SERVO_POS = HOOD_SERVO_MID;
                break;
            case MATH:
                HOOD_SERVO_POS = getDistanceToGoal() >= 100 ? HOOD_SERVO_MID : HOOD_SERVO_MAX;
                targetRPM = calculateShooterRPM(getDistanceToGoal());
                break;
            case STOP:
                targetRPM = ShooterConstants.IDLE_SHOOTER;
                HOOD_SERVO_POS = HOOD_SERVO_MIN;
                break;
            case TESTING:
                targetRPM = ShooterConstants.tuningRPM;
                HOOD_SERVO_POS = TESTING_HOOD_POS;
                break;

            case WHILE_MOVING:
                updateShotForVelocityComp();
                break;
        }
    }

    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public void setBangBangPower(double targetRPM){
        if (getShooterRPM() <= targetRPM){
            shooterMotor.setPower(1);
            shooterMotor2.setPower(1);
        } else if (getShooterRPM() > targetRPM){
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
        }

        PerTelem.addData("Shooter Current RPM", getShooterRPM());
        PerTelem.addData("Shooter Target RPM", targetRPM);
    }

    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = (ticksPerSecToRPM(topVelocity));

        pidController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);

        double power = pidController.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.KF * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);

        shooterMotor.setPower(power);
        shooterMotor2.setPower(power);

        PerTelem.addData("Shooter Current RPM", currentRPM);
        PerTelem.addData("Shooter Target RPM", targetRPM);
    }

    public ShooterState getState(){
        return state;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public void setHood(double hoodPos){
        hoodServo.setPosition(hoodPos);
    }

    public double calculateShooterRPM(double x) {
       return 1500;
    }

    public double getShooterRPM() {
        double velocity = shooterMotor.getVelocity();
        return (velocity * 60.0) / ShooterConstants.TICKS_PER_REV;
    }

    private void updateShotForVelocityComp() {
        double x = Math.max(Robot.compensatedDistance, 1);
        double y = ShooterConstants.TARGET_Y;
        double a = ShooterConstants.IMPACT_ANGLE_THETA;
        double g = ShooterConstants.G;

        double hoodAngle = Math.atan(2 * y / x - Math.tan(a));
        hoodAngle = Range.clip(hoodAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        double flywheelSpeed = Math.sqrt(
                g * x * x /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y))
        );

        double theoreticalRPM = (flywheelSpeed * 60) / (2 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS);
        targetRPM = (theoreticalRPM * ShooterConstants.VELOCITY_TO_RPM_RATIO) + ShooterConstants.RPM_BASE;

        double hoodServoPos = ShooterConstants.HOOD_SERVO_MIN +
                ((hoodAngle - ShooterConstants.HOOD_MIN_ANGLE) /
                        (ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE)) *
                        (ShooterConstants.HOOD_SERVO_MAX - ShooterConstants.HOOD_SERVO_MIN);

        hoodServoPos = Range.clip(hoodServoPos, ShooterConstants.HOOD_SERVO_MIN, ShooterConstants.HOOD_SERVO_MAX);
        hoodServo.setPosition(hoodServoPos);

        PerTelem.addData("Hood Angle (deg)", Math.toDegrees(hoodAngle));
        PerTelem.addData("Hood Servo Pos", hoodServoPos);
        PerTelem.addData("Compensated Distance", x);
        PerTelem.addData("Target RPM (VelComp)", targetRPM);
    }





    public boolean atTargetSpeed(){
        return Math.abs(targetRPM - getShooterRPM()) < error && getState() != ShooterState.STOP;
    }

    @Override
    public void periodic(){
        setState(state);
//        pidController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
        setBangBangPower(targetRPM);
        setHood(HOOD_SERVO_POS);
    }


    public enum ShooterState{
        CLOSE,
        STOP,
        FAR,

        WHILE_MOVING,

        TESTING,
        MATH




    }
}

