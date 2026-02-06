package org.firstinspires.ftc.teamcode.subsystem.New;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Constants.IntakeConstants;

public class Intake implements Subsystem {
    DcMotorEx intakeMotor, transferMotor;

    public IntakeState state;

    public Intake(DcMotorEx intakeMotor, DcMotorEx transferMotor){
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;

    }

    public void setState(IntakeState state){
        this.state = state;
        switch (state){
            case ON:
                intakeMotor.setPower(IntakeConstants.IntakeOn);
                transferMotor.setPower(IntakeConstants.transferPowerON);
                break;
            case REVERSE:
                intakeMotor.setPower(IntakeConstants.IntakeReverse);
                transferMotor.setPower(IntakeConstants.transferPowerRev);
                break;
            case OFF:
                intakeMotor.setPower(IntakeConstants.intakeOff);
                transferMotor.setPower(IntakeConstants.transferPowerOFF);
                break;
        }
    }

    public IntakeState getState(){
        return state;
    }
    public enum IntakeState{
        ON,
        OFF,
        REVERSE,
    }
}
