package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;

public class Robot {

    DcMotorEx shooterMotor, counterRoller;
    ShooterCMD shooter;

    Blocker blocker;

    Servo blockerServo;


    public Robot(HardwareMap hardwareMap){

        shooterMotor =  hardwareMap.get(DcMotorEx.class, "shooter");
        counterRoller =  hardwareMap.get(DcMotorEx.class, "CR");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        shooter = new ShooterCMD(shooterMotor,counterRoller);
        blocker = new Blocker(blockerServo);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CommandScheduler.getInstance().reset();

    }
}
