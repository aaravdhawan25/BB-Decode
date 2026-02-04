package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;

public class ShooterCMD implements Subsystem {
    Telemetry telemetry;

    public DcMotorEx shooterMotor,counterRoller;

    public Servo blocker;

    public PIDFController pidfController;
    public PIDFController CRpidfController;

    public ShooterState state = ShooterState.STOP;

    public ShooterCMD(DcMotorEx shooterMotor, DcMotorEx counterRoller, Servo blocker){
        this.blocker = blocker;
        this.shooterMotor = shooterMotor;
        this.counterRoller = counterRoller;
        pidfController = new PIDFController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KF);
        CRpidfController = new PIDFController(ShooterConstants.CR_KP, ShooterConstants.CR_KI, ShooterConstants.CR_KD, ShooterConstants.CR_KF);
    }
    
    public void setState(ShooterState state){
        this.state = state;
        switch (state){
            case CLOSE:

        }
    }

    public enum ShooterState{
        CLOSE,
        STOP,
        FAR,
        MATH




    }
}

