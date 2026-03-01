package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

@TeleOp(name = "Shooter Tuner")
public class ShooterTuning extends LinearOpMode {
    public void runOpMode(){
        PerTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, "BLUE");
        robot.shooter.state = ShooterCMD.ShooterState.TESTING;
        waitForStart();
        while (opModeIsActive()) {
            PerTelem.addLine("--- SHOOTER ---");
            PerTelem.addData("Target RPM", ShooterConstants.tuningRPM);
            PerTelem.addData("At RPM", robot.shooter.atTargetSpeed());
            PerTelem.addLine();
            robot.update();
        }
        robot.stop();
    }
}