package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.botconstants;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
@Config
public class Intaker implements Subsystem {
    public DcMotor intake, transfer;
    Telemetry telemetry;
    public double intakePower;

    Servo intakePiv;

    public static double intakeUp = 0;
    public static double intakeDown = 0;
    public static double intakeOff = 0;

    public static double intakeIntake = -1;

    public static double intakeTransfer = 0.5;

    public static double transferPowerON = -1;
    public static double transferPowerOFF = 0;

    boolean isIntaking = false;
    boolean isTransfer = false;
    private IntakeStates state = IntakeStates.IDLE;

    public enum IntakeStates{
        INTAKING,
        TRANSFER,
        IDLE
    }

    botconstants botconstants;


    public Intaker(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakePiv = hardwareMap.get(Servo.class, "pivServo");
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
        intakePiv.setPosition(intakeDown);// changePos based on irl
        intake.setPower(intakeTransfer);
        transfer.setPower(transferPowerON);
    }
    public void pivIntake(){
        intakePiv.setPosition(intakeUp);// changePos based on irl
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

    @Override
    public void update(){
        switch (state){
            case IDLE:
                intakePiv.setPosition(intakeUp);
                transfer.setPower(transferPowerOFF);
                intake.setPower(intakeOff);
                break;

            case INTAKING:
                intakePiv.setPosition(intakeUp);
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);
                break;
                
            case TRANSFER:
                intakePiv.setPosition(intakeDown);
                intake.setPower(intakeTransfer);
                transfer.setPower(transferPowerON);
                break;
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

    }
}