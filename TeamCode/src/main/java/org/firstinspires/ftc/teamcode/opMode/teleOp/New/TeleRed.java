package org.firstinspires.ftc.teamcode.opMode.teleOp.New;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.New.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.New.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.ShooterCMD;
import org.firstinspires.ftc.teamcode.subsystem.New.Turret;
import org.firstinspires.ftc.teamcode.subsystem.New.TurretCMD;
import org.firstinspires.ftc.teamcode.utils.PerTelem;

@TeleOp(name = "TeleOp Red")
public class TeleRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, "RED");
        Drivetrain drive = new Drivetrain(hardwareMap,telemetry);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        PerTelem.init(telemetry);

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.ON));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeCommand(robot, Intake.IntakeState.REVERSE));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ParallelCommandGroup(
                        new TransferCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(
                new ParallelCommandGroup(
                        new TransferCancelCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        new ShooterCommand(robot, ShooterCMD.ShooterState.CLOSE),
                        new TransferCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new ParallelCommandGroup(
                        new TransferCancelCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new TurretCommand(robot, TurretCMD.TurretState.MATH),
                        new ShooterCommand(robot, ShooterCMD.ShooterState.MATH)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(
                new ParallelCommandGroup(
                        new TurretCommand(robot, TurretCMD.TurretState.FORWARD),
                        new ShooterCommand(robot, ShooterCMD.ShooterState.STOP)
                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ParallelCommandGroup(
                        new TurretCommand(robot, TurretCMD.TurretState.WHILE_MOVING_TURRET),
                        new ShooterCommand(robot, ShooterCMD.ShooterState.WHILE_MOVING)
                )
        );

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(
                new ParallelCommandGroup(
                        new TurretCommand(robot, TurretCMD.TurretState.FORWARD),
                        new ShooterCommand(robot, ShooterCMD.ShooterState.STOP)
                )
        );

        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                new TurretCommand(robot, TurretCMD.TurretState.FORWARD)
        ));
        waitForStart();

        while (opModeIsActive()){
            robot.update();
            drive.updateCtrls(gamepad1, gamepad2);

        }
        robot.stop();


    }
}
