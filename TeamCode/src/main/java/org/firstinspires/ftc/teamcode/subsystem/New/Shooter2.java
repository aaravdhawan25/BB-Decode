package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Shooter2 implements Subsystem {
    public static double KP = 0.0005;
    public static double KI = 0.0;
    public static double KD = 0.00001;
    public static double KF = 0.0002;

    public static double CR_KP = 0.00005;
    public static double CR_KI = 0.0;
    public static double CR_KD = 0.000000001;
    public static double CR_KF = 0.0001;

    public static double IDLE_SHOOTER = 1000;

    public static double TICKS_PER_REV = 28.0;
    public static double CR_TICKS_PER_REV = 28.0;
    public static double CLOSE_SHOOTER_RPM = 3450;
    public static double CLOSE_CR_RPM = 2700;
    public static double FAR_SHOOTER_RPM = 5100;
    public static double FAR_CR_RPM = 2600;
    public static double RPM_TOLERANCE = 50;
    public static double MAX_RPM = 5400;


    public static double RPM_BASE = 2500;
    public static double RPM_PER_INCH = 10.0;

    private double distanceToGoal = 0;

    public static double BlockerOpen = 0.5;

    public static double BlockerClosed = 1;

    Telemetry telemetry;

    private DcMotorEx shooterMotor;
    private DcMotorEx counterRoller;

    private Servo blocker;

    private MultipleTelemetry multipleTelemetry;

    private double targetRPM = 0;
    private double crTargetRPM = 0;

    private double integral = 0;
    private double lastError = 0;

    private double crIntegral = 0;
    private double crLastError = 0;

    private long lastTime = 0;

    public PIDFController pidfController;
    public PIDFController CRpidfController;

    private ShooterState state = ShooterState.IDLE;



    public enum ShooterState {
        IDLE,
        SPINNING_UP_CLOSE,
        SPINNING_UP_FAR,
        READY_CLOSE,
        READY_FAR,

        SPINNING_UP_AUTO,
        READY_AUTO,
        OPEN_BLOCKER,
        STOPPING
    }

    public Shooter2(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pidfController = new PIDFController(KP,KI,KD,KF);
        CRpidfController = new PIDFController(CR_KP, CR_KI,CR_KD, CR_KF);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        counterRoller = hardwareMap.get(DcMotorEx.class, "CR");
        blocker = hardwareMap.get(Servo.class, "blocker");

    }

    @Override
    public void init() {
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blockerClose();
        telemetry.addData("Shooter", "Initialized");
        telemetry.addData("Counter Roller", "Initialized");
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

    public double getCounterRollerRPM() {
        double velocity = counterRoller.getVelocity();
        return (velocity * 60.0) / CR_TICKS_PER_REV;
    }

    public boolean atTargetSpeed() {
        double shooterRPM = getShooterRPM();
        double crRPM = getCounterRollerRPM();
        return Math.abs(targetRPM - shooterRPM) < RPM_TOLERANCE &&
                Math.abs(crTargetRPM - crRPM) < RPM_TOLERANCE;
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

    public double CalculateCRError(){
        double CRError = crTargetRPM-getCounterRollerRPM();

        return CRError;
    }
    public void kickBallOff(){
        setState(ShooterState.OPEN_BLOCKER);
    }


    public double calculateShooterRPM(double distance) {
        return RPM_BASE + (RPM_PER_INCH * distance);
    }

    public double calculateCounterRollerRPM(double distance) {
        return RPM_BASE + (RPM_PER_INCH * distance);
    }

    public void spinUpAuto() {
        setState(ShooterState.SPINNING_UP_AUTO);
    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        switch (state) {
            case IDLE:
                targetRPM = IDLE_SHOOTER;
                crTargetRPM = 0;
                blockerClose();
//                resetPID();
                break;

            case SPINNING_UP_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                crTargetRPM = CLOSE_CR_RPM;
                blockerClose();

                if (atTargetSpeed()) {
                    setState(ShooterState.READY_CLOSE);
                }
                break;

            case SPINNING_UP_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                crTargetRPM = FAR_CR_RPM;
                blockerClose();
                if (atTargetSpeed()) {
                    setState(ShooterState.READY_FAR);
                }
                break;

            case READY_CLOSE:
                targetRPM = CLOSE_SHOOTER_RPM;
                crTargetRPM = CLOSE_CR_RPM;
                blockerOpen();
                break;

            case READY_FAR:
                targetRPM = FAR_SHOOTER_RPM;
                crTargetRPM = FAR_CR_RPM;
                blockerOpen();

                break;

            case SPINNING_UP_AUTO:
                double distance = getDistanceToGoal();
                targetRPM = calculateShooterRPM(distance);
                crTargetRPM = calculateCounterRollerRPM(distance);
                blockerClose();
                if (atTargetSpeed()) {
                    setState(ShooterState.READY_AUTO);
                }
                break;

            case READY_AUTO:
                distance = getDistanceToGoal();
                targetRPM = calculateShooterRPM(distance);
                crTargetRPM = calculateCounterRollerRPM(distance);
                blockerOpen();
//                intake.Transfer();
                break;
            case OPEN_BLOCKER:
                blockerOpen();
                break;

            case STOPPING:
                targetRPM = IDLE_SHOOTER;
                crTargetRPM = 0;
                blockerClose();

                if (getShooterRPM() < 100 && getCounterRollerRPM() < 100) {
                    setState(ShooterState.IDLE);
                }
                break;
        }

        updateShooterMotor(dt);
        updateCounterRollerMotor(dt);

        telemetry.addData("State", state);
        telemetry.addData("Shooter Current RPM", getShooterRPM());
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Counter Roller Current RPM", Math.abs(getCounterRollerRPM()));
        telemetry.addData("Counter Roller Target RPM", crTargetRPM);
        telemetry.addData("At Speed", atTargetSpeed());
        telemetry.addData("ShooterPower", shooterMotor.getPower());
        telemetry.addData("CR Power", counterRoller.getPower());
        telemetry.addData("Shooter Error", calculateError());
        telemetry.addData("CR Error", CalculateCRError());
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        crIntegral = 0;
        crLastError = 0;
    }

    private void updateShooterMotor(double dt) {
        double currentRPM = getShooterRPM();
        double error = targetRPM - currentRPM;
        pidfController.setPIDF(KP,KI,KD,KF);
        double power = pidfController.calculate(currentRPM,targetRPM);
        power = Range.clip(power, 0, 1);

        shooterMotor.setPower(power);
    }

    private void updateCounterRollerMotor(double dt) {
        double currentRPMCR = getCounterRollerRPM();
        double error = crTargetRPM + currentRPMCR;
        CRpidfController.setPIDF(CR_KP,CR_KI,CR_KD, CR_KF);
        double CRpower = CRpidfController.calculate(-currentRPMCR, crTargetRPM);
        CRpower = Range.clip(CRpower, 0, 1);

        counterRoller.setPower(CRpower);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.xWasPressed()) {
            spinUpClose();
        }
        if (gp1.xWasReleased()){
            stop();
        }

        if (gp1.yWasPressed()) {
            spinUpFar();
        }

        if (gp1.yWasReleased()){
            stop();
        }

        if (gp2.aWasPressed()){
            spinUpAuto();
        }
        if (gp2.aWasReleased()){
            stop();
        }

        if (state == ShooterState.READY_FAR){
            gp1.rumble(200);
        }
        if (state == ShooterState.READY_CLOSE){
            gp1.rumble(200);
        }

        if (state != ShooterState.READY_FAR){
            gp1.stopRumble();
        }
        if (state != ShooterState.READY_CLOSE){
            gp1.stopRumble();
        }

        if (gp1.dpadDownWasPressed()){
            kickBallOff();
        }
        if (gp1.dpadDownWasReleased()){
            stop();
        }



    }
}