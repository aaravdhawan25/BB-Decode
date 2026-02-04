package org.firstinspires.ftc.teamcode.subsystem.New;

import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.crTargetRPM;
import static org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants.targetRPM;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;

@Config
public class ShooterCMD implements Subsystem {
    Telemetry telemetry;

    public DcMotorEx shooterMotor,counterRoller;
    private double distanceToGoal = 0;
    public PIDFController pidfController;
    public PIDFController CRpidfController;

    public ShooterState state = ShooterState.STOP;

    public long lastTime = 0;

    public ShooterCMD(DcMotorEx shooterMotor, DcMotorEx counterRoller){
        this.shooterMotor = shooterMotor;
        this.counterRoller = counterRoller;
        pidfController = new PIDFController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KF);
        CRpidfController = new PIDFController(ShooterConstants.CR_KP, ShooterConstants.CR_KI, ShooterConstants.CR_KD, ShooterConstants.CR_KF);
    }
    
    public void setState(ShooterState state){
        this.state = state;
        switch (state){
            case CLOSE:
                targetRPM = ShooterConstants.CLOSE_SHOOTER_RPM;
                crTargetRPM = ShooterConstants.CLOSE_CR_RPM;
                break;
            case FAR:
                targetRPM = ShooterConstants.FAR_SHOOTER_RPM;
                crTargetRPM = ShooterConstants.FAR_CR_RPM;
                break;
            case MATH:
                targetRPM = calculateShooterRPM(getDistanceToGoal());
                crTargetRPM = targetRPM*ShooterConstants.CR_RATIO;
                break;
            case STOP:
                targetRPM = ShooterConstants.IDLE_SHOOTER;
                crTargetRPM = 0;
        }
    }

    private void updateShooterMotor(double dt) {
        double currentRPM = getShooterRPM();
        double error = targetRPM - currentRPM;
        double power = pidfController.calculate(currentRPM,targetRPM);
        power = Range.clip(power, 0, 1);

        shooterMotor.setPower(power);
    }

    private void updateCounterRollerMotor(double dt) {
        double currentRPMCR = getCounterRollerRPM();
        double error = crTargetRPM + currentRPMCR;
        double CRpower = CRpidfController.calculate(-currentRPMCR, crTargetRPM);
        CRpower = Range.clip(CRpower, 0, 1);

        counterRoller.setPower(CRpower);
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

    public double getCounterRollerRPM() {
        double velocity = counterRoller.getVelocity();
        return (velocity * 60.0) / ShooterConstants.CR_TICKS_PER_REV;
    }

    public boolean atTargetSpeed() {
        double shooterRPM = getShooterRPM();
        double crRPM = getCounterRollerRPM();
        return Math.abs(targetRPM - shooterRPM) < ShooterConstants.RPM_TOLERANCE &&
                Math.abs(crTargetRPM - crRPM) < ShooterConstants.RPM_TOLERANCE;
    }

    @Override
    public void periodic(){
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        setState(state);
        pidfController.setPIDF(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KF);
        CRpidfController.setPIDF(ShooterConstants.CR_KP, ShooterConstants.CR_KI, ShooterConstants.CR_KD, ShooterConstants.CR_KF);
        updateShooterMotor(dt);
        updateCounterRollerMotor(dt);

    }


    public enum ShooterState{
        CLOSE,
        STOP,
        FAR,
        MATH




    }
}

