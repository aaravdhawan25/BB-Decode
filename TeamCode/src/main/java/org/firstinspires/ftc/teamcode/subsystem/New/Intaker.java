package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.botconstants;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

import java.security.PublicKey;

@Config
public class Intaker implements Subsystem {
    public DcMotor intake, transfer;
    Telemetry telemetry;
    public double intakePower;


    public static double intakeUp = 0;
    public static double intakeDown = 0;
    public static double intakeOff = 0;

    public static double intakeIntake = -1;

    public static double intakeTransfer = 0.5;

    public static double intakeReverse = 1;

    public static double transferPowerON = -1;
    public static double transferPowerOFF = 0;
    public static double transferPowerRev = 1;

    private IntakeStates state = IntakeStates.IDLE;

    public enum IntakeStates{
        INTAKING,
        TRANSFER,
        REVERSE,

        ReverseWith,
        IDLE
    }

    botconstants botconstants;


    public Intaker(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }


    @Override
    public void init() {
        telemetry.addData("Intake","Initialized");
        telemetry.addData("PivServo", "Initialized");
        telemetry.addData("Transfer", "Initialized");
        telemetry.update();

    }


    public void autoIntake(){
        intake.setPower(intakeIntake);
        transfer.setPower(transferPowerON);
    }

    public void autoIntakeOff(){
        intake.setPower(intakeOff);
        transfer.setPower(transferPowerOFF);
    }

    public void pivSendBalls(){
        intake.setPower(intakeTransfer);
        transfer.setPower(transferPowerON);
    }
    public void pivIntake(){
        transfer.setPower(transferPowerOFF);
    }

    public void setState(Intaker.IntakeStates newState) {
        this.state = newState;
    }

    public void IntakeIdle(){
        setState(IntakeStates.IDLE);

    }

    public void Intake(){
        setState(IntakeStates.INTAKING);

    }

    public void Transfer(){
        setState(IntakeStates.TRANSFER);

    }

    public void Reverse(){
        setState(IntakeStates.REVERSE);
    }
    public void Rev(){
        setState(IntakeStates.ReverseWith);
    }

    @Override
    public void update(){
        switch (state){
            case IDLE:
                transfer.setPower(transferPowerOFF);
                intake.setPower(intakeOff);
                break;

            case INTAKING:
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);
                break;

            case TRANSFER:
                intake.setPower(intakeTransfer);
                transfer.setPower(transferPowerON);
                break;
            case REVERSE:
                intake.setPower(intakeReverse);
            case ReverseWith:
                intake.setPower(intakeReverse);
                transfer.setPower(transferPowerRev);

        }

        telemetry.addData("Intake State", state);

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp2.aWasPressed()){
            Transfer();
        }

        if (gp2.aWasReleased()){
            IntakeIdle();
        }


        if (gp1.rightBumperWasPressed()){
            Intake();
        }
        if (gp1.rightBumperWasReleased()){
            IntakeIdle();
        }

        if (gp1.leftBumperWasPressed()){
            Reverse();
        }

        if (gp1.leftBumperWasReleased()){
            IntakeIdle();
        }

        if (gp1.bWasPressed()){
            Rev();
        }
        if (gp1.bWasReleased()){
            IntakeIdle();
        }

    }
}