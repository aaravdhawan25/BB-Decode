package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.CustomAdaptiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;
import org.firstinspires.ftc.teamcode.subsystem.New.Turret;

@TeleOp(name = "COMP Teleop", group = "a")
public class prod_TestTele extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));



    Drivetrain drivetrain;
    CustomAdaptiveIntake customAdaptiveIntake;
    Shooter2 shooter;

    boolean isEndgame = false;

    MecanumDrive follower;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap,telemetry);
        customAdaptiveIntake = new CustomAdaptiveIntake(hardwareMap, telemetry);
        shooter = new Shooter2(hardwareMap, telemetry);
        follower = new MecanumDrive(hardwareMap, START_POSE);
        drivetrain.init();
        customAdaptiveIntake.init();
        shooter.init();


        while (!opModeIsActive()) {
            telemetry.addData("Status", "Waiting for Start");
            runtime.reset();

        }

        waitForStart();

        while (opModeIsActive()){
            runtime.startTime();



            drivetrain.update();
            customAdaptiveIntake.update();
            shooter.update();


            drivetrain.updateCtrls(gamepad1, gamepad2);
            customAdaptiveIntake.updateCtrls(gamepad1, gamepad2);
            shooter.updateCtrls(gamepad1, gamepad2);

            if (runtime.seconds() >= 140){
                gamepad1.rumble(200);
                isEndgame = true;
            }

            telemetry.addData("Time Period", isEndgame ? "TeleOp" : "Go Park");
            
            telemetry.update();
        }
    }
}
