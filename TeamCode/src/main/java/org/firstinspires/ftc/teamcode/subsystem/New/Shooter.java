package org.firstinspires.ftc.teamcode.subsystem.New;

import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MID;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.hoodServoPosition;

import android.graphics.Camera;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

@Config
public class Shooter implements Subsystem {
    public static double KP = 0.0005;
    public static double KI = 0.0;
    public static double KD = 0.00001;
    public static double KF = 0.0002;

    public boolean isOneMan = false;

    public static double CR_KP = 0.00005;
    public static double CR_KI = 0.0;
    public static double CR_KD = 0.000000001;
    public static double CR_KF = 0.0001;

    public static double IDLE_SHOOTER = 2500;

    public static double TICKS_PER_REV = 28.0;
    public static double CR_TICKS_PER_REV = 28.0;
    public static double CLOSE_SHOOTER_RPM = 3000;

    public static double AUTO_SHOOTER_RPM = 2750;
    public static double AUTO_CR_RPM = 2650;
    public static double CLOSE_CR_RPM = 2700;
    public static double FAR_SHOOTER_RPM = 4000;
    public static double FAR_CR_RPM = 2150;
    public static double RPM_TOLERANCE = 100;

    public static double G = 386.1;
    public static double TARGET_Y = 25.5;
    public static double IMPACT_ANGLE_THETA = -0.002;
    public static double FLYWHEEL_RADIUS = 1.43701;

    public static double CR_RATIO = 0.7;
    public static double VELOCITY_TO_RPM_RATIO = 2.05;
    public static double RPM_BASE = 400.0;




    public static double distanceToGoal = 0;

    public static double BlockerOpen = 1;

    public static double BlockerClosed = 0.83;

    Telemetry telemetry;

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;

    private Servo blocker;


    public static double targetRPM = 0;
    private double crTargetRPM = 0;

    private double integral = 0;
    private double lastError = 0;

    private double crIntegral = 0;
    private double crLastError = 0;

    private long lastTime = 0;

    public PIDFController pidfController;
    public PIDFController CRpidfController;

    private ShooterState state = ShooterState.IDLE;

    Servo hoodServo;



    public enum ShooterState {
        IDLE,
        SPINNING_UP_CLOSE,
        SPINNING_UP_FAR,
        READY_CLOSE,
        READY_FAR,
        SPINNING_UP_DISTANCE,
        READY_DISTANCE,

        OFF,

        SPINNING_UP_AUTO,
        READY_AUTO,
        OPEN_BLOCKER,
        STOPPING
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blocker = hardwareMap.get(Servo.class, "blocker");

    }

    @Override
    public void init() {
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blockerClose();
        telemetry.addData("Shooter", "Initialized");
        telemetry.addData("Blocker", "Initialized");
        telemetry.update();
        lastTime = System.nanoTime();
    }

    public void blockerOpen() {
        blocker.setPosition(BlockerOpen);
    }

    public void blockerClose() {
        blocker.setPosition(BlockerClosed);
    }

    public void setState(ShooterState newState) {
        this.state = newState;
    }

    public ShooterState getState() {
        return state;
    }

    public void spinUpClose() {
        setState(ShooterState.SPINNING_UP_CLOSE);
    }

    public void spinUpFar() {
        setState(ShooterState.SPINNING_UP_FAR);
    }

    public void stop() {
        setState(ShooterState.STOPPING);
    }

    public double getShooterRPM() {
        double velocity = shooterMotor.getVelocity();
        return (velocity * 60.0) / TICKS_PER_REV;
    }

    public boolean atTargetSpeed() {
        double shooterRPM = getShooterRPM();
        return Math.abs(targetRPM - shooterRPM) < RPM_TOLERANCE;
    }

    public void setDistanceToGoal(double distance) {
        this.distanceToGoal = distance;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double calculateError(){
        double ShooterError = targetRPM-getShooterRPM();

        return ShooterError;
    }


    public void kickBallOff(){
        setState(ShooterState.OPEN_BLOCKER);
    }


    public double calculateShooterRPM(double x) {
        if (x < 1.0) x = 1.0 ;

        double alpha = Math.atan((2 * TARGET_Y / x) - Math.tan(IMPACT_ANGLE_THETA));

        double cosAlpha = Math.cos(alpha);
        double v0 = Math.sqrt(
                (G * Math.pow(x, 2)) /
                        (2 * Math.pow(cosAlpha, 2) * (x * Math.tan(alpha) - TARGET_Y))
        );

        double theoreticalRPM = (v0 * 60) / (2 * Math.PI * FLYWHEEL_RADIUS);
        return (theoreticalRPM * VELOCITY_TO_RPM_RATIO) + RPM_BASE;
    }

    public double setHood(double angleRad) {

        double minAngle = ShooterConstants.HOOD_MIN_ANGLE; //0.2
        double maxAngle = ShooterConstants.HOOD_MAX_ANGLE; //0.3
        double servoHigh = 0.3;
        double servoLow = 0.2;
        double loA = Math.min(minAngle, maxAngle);
        double hiA = Math.max(minAngle, maxAngle);
        angleRad = MathUtils.clamp(angleRad, loA, hiA);
        hoodServoPosition= servoLow + (angleRad - loA) * (servoHigh - servoLow) / (hiA - loA);
        return hoodServoPosition;
    }

    public double getHood(){
        double hoodAngle = Math.atan(2.0 * TARGET_Y / (getDistanceToGoal()-PASS_THROUGH_POINT_RADIUS) - Math.tan(IMPACT_ANGLE_THETA));
        telemetry.addData("HOOD ANGLE RAW (no vel comp)", hoodAngle);
        hoodAngle = MathUtils.clamp(hoodAngle,
                ShooterConstants.HOOD_MIN_ANGLE,
                ShooterConstants.HOOD_MAX_ANGLE);


        return hoodAngle;
    }
    private boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static double[] lastGood = new double[]{
            ShooterConstants.HOOD_MIN_ANGLE,
            0.0                            // flywheel speed
    };

    public double getFlywheelSpeed(){
        double term = (getDistanceToGoal() - PASS_THROUGH_POINT_RADIUS) * Math.tan(getHood()) - TARGET_Y;
        double cos = Math.cos(getHood());
        double denom1 = 2 * cos * cos * term;
        double flyWheelSpeed = Math.sqrt(G * (getDistanceToGoal()-PASS_THROUGH_POINT_RADIUS) * (getDistanceToGoal()-PASS_THROUGH_POINT_RADIUS) / denom1);
        return flyWheelSpeed;
    }

    public double getRPM(double distanceToGoal){
        return -0.105746 * distanceToGoal*distanceToGoal +32.33112*distanceToGoal +1558.21332;
    }

    public double getHoodPos(double distanceToGoal){
        double hoodPos = HOOD_SERVO_MIN;
        if (distanceToGoal >= 100){
            hoodPos = HOOD_SERVO_MID;
        } else {
            hoodPos = HOOD_SERVO_MAX;
        }

        if (LLCam.target == null) {
            hoodPos = hoodServo.getPosition();
        } else if(!LLCam.target.hasTarget){
            hoodPos = hoodServo.getPosition();
        }

        return hoodPos;
    }

    public void spinUpAuto() {
        setState(ShooterState.SPINNING_UP_AUTO);
    }

    public void spinUpDistance(){
        setState(ShooterState.SPINNING_UP_DISTANCE);
    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        switch (state) {
            case IDLE:
                targetRPM = IDLE_SHOOTER;
                hoodServoPosition = HOOD_SERVO_MIN;
                blockerClose();
//                resetPID();
                break;
            case SPINNING_UP_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MAX;
                blockerClose();

                if (atTargetSpeed()) {
                    setState(ShooterState.READY_CLOSE);
                }
                break;
            case SPINNING_UP_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MID;
                blockerClose();
                if (atTargetSpeed()) {
                    setState(ShooterState.READY_FAR);
                }
                break;
            case READY_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MAX;
                blockerOpen();
                break;
            case READY_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MID;
                blockerOpen();
                break;
            case SPINNING_UP_AUTO:
                targetRPM = AUTO_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MID;
                blockerClose();
                if (atTargetSpeed()) {
                    setState(ShooterState.READY_AUTO);
                }
                break;
            case READY_AUTO:
                targetRPM = AUTO_SHOOTER_RPM;
                hoodServoPosition = HOOD_SERVO_MID;
                blockerOpen();
                break;
            case OPEN_BLOCKER:
                blockerOpen();
                break;
            case SPINNING_UP_DISTANCE:
                double rpm = getRPM(CameraConstants.distanceToGoalLL);
                hoodServoPosition = getHoodPos(CameraConstants.distanceToGoalLL);
                targetRPM = rpm;
                telemetry.addData("HOOD POSITION", hoodServoPosition);
                telemetry.addData("RPM", rpm);
                if (atTargetSpeed()){
                    setState(ShooterState.READY_DISTANCE);
                }

//                targetRPM = calculateShooterRPM(getDistanceToGoal());
//                hoodServoPosition = setHood(getHood());
                break;
            case READY_DISTANCE:
                targetRPM = getRPM(CameraConstants.distanceToGoalLL);
                hoodServoPosition = getHoodPos(CameraConstants.distanceToGoalLL);
                blockerOpen();
                break;
            case STOPPING:
                targetRPM = IDLE_SHOOTER;
                hoodServoPosition = HOOD_SERVO_MIN;
                blockerClose();
                if (getShooterRPM() < 100 ) {
                    setState(ShooterState.IDLE);
                }
                break;
        }

        updateShooterMotor();

        hoodServo.setPosition(hoodServoPosition);

        telemetry.addData("State", state);
        telemetry.addData("Shooter Current RPM", getShooterRPM());
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Counter Roller Target RPM", crTargetRPM);
        telemetry.addData("At Speed", atTargetSpeed());
        telemetry.addData("ShooterPower", shooterMotor.getPower());
        telemetry.addData("CR Power", shooterMotor2.getPower());
        telemetry.addData("Shooter Error", calculateError());
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        crIntegral = 0;
        crLastError = 0;
    }

    public void setBangBangPower(double targetRPM){
        if (getShooterRPM() <= targetRPM){
            shooterMotor.setPower(1);
            shooterMotor2.setPower(1);
        } else if (getShooterRPM() > targetRPM){
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
        }

        telemetry.addData("Shooter Current RPM", getShooterRPM());
        telemetry.addData("Shooter Target RPM", targetRPM);
    }

    private void updateShooterMotor() {
        setBangBangPower(targetRPM);

    }


    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.rightStickButtonWasPressed()){
            isOneMan = !isOneMan;

        }

        if (isOneMan) {

            if (gp1.dpadDownWasPressed()) {
                kickBallOff();
            }
            if (gp1.dpadDownWasReleased()) {
                stop();
            }
            if (gp1.triangleWasPressed()) {
                spinUpDistance();
            }
            if (gp1.triangleWasReleased()) {
                stop();
            }
            if (gp1.dpadUpWasPressed()){
                spinUpFar();
            }
            if (gp1.dpadUpWasReleased()){
                stop();
            }

            if (gp1.right_trigger >= 0.1){
                spinUpClose();
            }

            if (gp1.right_trigger < 0.1){
                stop();
            }
        } else {
            if (gp2.xWasPressed()) {
                spinUpClose();
            }
            if (gp2.xWasReleased()) {
                stop();
            }
            if (gp1.dpadDownWasPressed()) {
                kickBallOff();
            }
            if (gp1.dpadDownWasReleased()) {
                stop();
            }
            if (gp2.triangleWasPressed()) {
                spinUpDistance();
            }
            if (gp2.triangleWasReleased()) {
                stop();
            }
            if (gp2.dpadUpWasPressed()){
                spinUpFar();
            }
            if (gp2.dpadUpWasReleased()){
                stop();
            }
        }



    }
}