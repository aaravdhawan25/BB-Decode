package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.Intaker;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;

@TeleOp(name = "COMP Teleop", group = "a")
public class prod_TestTele extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));



    Drivetrain drivetrain;
    Intaker intaker;
    Shooter2 shooter;

    boolean isEndgame = false;

    MecanumDrive follower;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap,telemetry);
        intaker = new Intaker(hardwareMap, telemetry);
        shooter = new Shooter2(hardwareMap, telemetry);
        follower = new MecanumDrive(hardwareMap, START_POSE);
        drivetrain.init();
        intaker.init();
        shooter.init();


        while (opModeInInit()) {
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
            runtime.reset();

        }

        waitForStart();

        while (opModeIsActive()){
            runtime.startTime();



            drivetrain.update();
            intaker.update();
            shooter.update();


            drivetrain.updateCtrls(gamepad1, gamepad2);
            intaker.updateCtrls(gamepad1, gamepad2);
            shooter.updateCtrls(gamepad1, gamepad2);

            if (runtime.seconds() >= 140) {
                isEndgame = true;
            } else {
                isEndgame = false;}

            telemetry.addData("Time Period", isEndgame ? "TeleOp" : "Go Park");

            telemetry.update();
        }
    }
}
