package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
@Config
public class kickStand implements Subsystem {
    Servo kickStand;

    double kickDown = 1;

    double kickUp = 0;

    Telemetry telemetry;

    public kickStand(HardwareMap map, Telemetry telem){
        kickStand = map.get(Servo.class, "kickStand");
        this.telemetry = telem;
    }

    @Override
    public void init() {
        setKickUp();


    }

    public void setKickUp(){
        kickStand.setPosition(kickUp);
    }

    public void setKickDown(){
        kickStand.setPosition(kickDown);
    }

    @Override
    public void update() {

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        boolean down = false;

        if (gp1.dpadLeftWasPressed()){
            telemetry.addData("Kick State","Down" );
            setKickDown();
        }

        if (gp1.dpadLeftWasReleased()){
            setKickUp();
        }

    }
}
