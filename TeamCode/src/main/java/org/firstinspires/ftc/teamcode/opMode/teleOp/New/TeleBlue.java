package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.AlignCommand;
import org.firstinspires.ftc.teamcode.robot.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.LLCam;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

@TeleOp(name = "TeleOp Blue")
public class TeleBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, "BLUE");
        Drivetrain drive = new Drivetrain(hardwareMap,telemetry);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        PerTelem.init(telemetry);

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.ON));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.REVERSE));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new AlignCommand(robot, LLCam.alignState.MATH_CAMERA)
        );

        gp1.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new AlignCommand(robot, LLCam.alignState.STOP)
        );


        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new TransferCommand(robot, ShooterCMD.ShooterState.CLOSE)
        );
        gp2.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new TransferCancelCommand(robot)
        );
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                        new TransferCommand(robot, ShooterCMD.ShooterState.FAR)
        );
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(
                new ParallelCommandGroup(
                        new TransferCancelCommand(robot)
                )
        );


        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
        ));
        waitForStart();

        while (opModeIsActive()){
            robot.update();
            if (!gamepad1.x){
                drive.updateCtrls(gamepad1, gamepad2);

            }
        }
        robot.stop();


    }
}
