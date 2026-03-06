package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTesters extends LinearOpMode {

    public static String hwName = "name";
    Servo servo;

    public static double servoPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, hwName);


        waitForStart();
        while (opModeIsActive()){
            servo.setPosition(servoPos);
            telemetry.addData("Servo Pos", servoPos);
            telemetry.update();
        }
    }
}
