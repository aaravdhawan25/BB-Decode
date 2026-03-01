package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "Shooter & Hood Tester")
public class ShooterMotorTester extends LinearOpMode {
    DcMotorEx shooterMotor, shooterMotor2;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor =  hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor2 =  hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            shooterMotor.setPower(1);
            shooterMotor2.setPower(1);
        }
    }
}
