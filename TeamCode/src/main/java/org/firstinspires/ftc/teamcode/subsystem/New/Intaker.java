package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Intaker implements Subsystem {
    public DcMotor intake, transfer;
    Telemetry telemetry;
    public double intakePower;

    RevColorSensorV3 disColor;

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

        INTAKE_ONLY,

        CHECK,

        MANUALIN,
        TRANSFER,

        REVERSE,
        ReverseWith,

        TRANSFER_SHOOTER,
        IDLE
    }



    public Intaker(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        disColor = hardwareMap.get(RevColorSensorV3.class, "distSensor");
    }


    @Override
    public void init() {
        telemetry.addData("Intake","Initialized");
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

    public void check(){
        setState(IntakeStates.CHECK);
    }

    public double getDist(){
        return disColor.getDistance(DistanceUnit.CM);
    }

    public void Intake(){
        setState(IntakeStates.INTAKING);

    }

    public void Transfer(){
        setState(IntakeStates.TRANSFER);

    }

    public IntakeStates getState(){
        return state;
    }

    public void Reverse(){
        setState(IntakeStates.REVERSE);
    }
    public void Rev(){
        setState(IntakeStates.ReverseWith);
    }

    public void TransferShootSub(){
        setState(IntakeStates.TRANSFER_SHOOTER);
    }

    public void manualIn(){
        setState(IntakeStates.MANUALIN);
    }

    @Override
    public void update(){
        switch (state){
            case IDLE:
                transfer.setPower(transferPowerOFF);
                intake.setPower(intakeOff);
                break;
            case MANUALIN:
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);
                break;

            case INTAKING:
                if (getDist() < 3){
                    setState(IntakeStates.INTAKE_ONLY);
                }
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);

                break;
            case INTAKE_ONLY:
                if (getDist() > 5){
                    setState(IntakeStates.INTAKING);
                }
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerOFF);

                break;

            case TRANSFER:
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);
                break;
            case REVERSE:
                intake.setPower(intakeReverse);
                break;
            case ReverseWith:
                intake.setPower(intakeReverse);
                transfer.setPower(transferPowerRev);
                break;
            case TRANSFER_SHOOTER:
                intake.setPower(intakeIntake);
                transfer.setPower(transferPowerON);
                break;


        }

        telemetry.addData("Intake State", state);
        telemetry.addData("Distance Sensor Dist", getDist());

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.aWasPressed()){
            Transfer();
        }

        if (gp1.aWasReleased()){
            IntakeIdle();
        }


        if (gp1.rightBumperWasPressed()){
            Intake();
        }

        if (gp1.right_bumper){
            setState(getState());
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