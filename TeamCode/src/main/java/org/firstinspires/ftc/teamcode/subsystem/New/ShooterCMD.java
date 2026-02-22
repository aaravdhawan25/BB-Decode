package org.firstinspires.ftc.teamcode.subsystem.New;

import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.crTargetRPM;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.targetRPM;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

@Config
public class ShooterCMD implements Subsystem {
    Telemetry telemetry;

    public DcMotorEx shooterMotor,shooterMotor2;

    Servo hoodServo;
    private double distanceToGoal = 0;
    public PIDController pidController;

    public ShooterState state = ShooterState.STOP;

    public long lastTime = 0;

    public ShooterCMD(DcMotorEx shooterMotor, DcMotorEx shooterMotor2, Servo hoodServo){
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
        this.hoodServo = hoodServo;
        pidController = new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
        pidController.setTolerance(10);
    }
    
    public void setState(ShooterState state){
        this.state = state;
        switch (state){
            case CLOSE:
                targetRPM = ShooterConstants.CLOSE_SHOOTER_RPM;
            case FAR:
                targetRPM = ShooterConstants.FAR_SHOOTER_RPM;
                break;
            case MATH:
                targetRPM = calculateShooterRPM(getDistanceToGoal());
                break;
            case STOP:
                targetRPM = ShooterConstants.IDLE_SHOOTER;
                break;
            case TESTING:
                targetRPM = ShooterConstants.tuningRPM;
                break;

            case WHILE_MOVING:
                updateShotForVelocityComp();
                break;
        }
    }

    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
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

    public void setDistanceToGoal(double distance) {
        this.distanceToGoal = distance;

    }

    public double calculateShooterRPM(double x) {
        if (x < 1.0) x = 1.0 ;

        double alpha = Math.atan((2 * ShooterConstants.TARGET_Y / x) - Math.tan(ShooterConstants.IMPACT_ANGLE_THETA));

        double cosAlpha = Math.cos(alpha);
        double v0 = Math.sqrt(
                (ShooterConstants.G * Math.pow(x, 2)) /
                        (2 * Math.pow(cosAlpha, 2) * (x * Math.tan(alpha) - ShooterConstants.TARGET_Y))
        );

        double theoreticalRPM = (v0 * 60) / (2 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS);

        return (theoreticalRPM * ShooterConstants.VELOCITY_TO_RPM_RATIO) + ShooterConstants.RPM_BASE;
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





    public boolean atTargetSpeed() {
        return pidController.getPositionError() <= 100;
    }

    @Override
    public void periodic(){
        setState(state);
        pidController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
        setShooterPIDPower(targetRPM);

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

