package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ResetArmCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

@TeleOp
public class BasicOpMode extends CommandOpMode {
    private DriveSubsystem m_driveSubsystem;
    private ClawSubsystem m_clawSubsystem;
    private ArmSubsystem m_armSubsystem;
    private DroneSubsystem m_droneSubsystem;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx mechGamepad = new GamepadEx(gamepad2);

        /* SUBSYSTEMS */
        //send motor and sensor info from DriverStation config to subsystems
        m_driveSubsystem = new DriveSubsystem(
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "backLeft"),
                hardwareMap.get(DcMotor.class, "backRight"),
                hardwareMap.get(BNO055IMU.class, "imu"),
                telemetry
        );

        m_clawSubsystem = new ClawSubsystem(
                hardwareMap.get(Servo.class, "armServo"),
                hardwareMap.get(Servo.class, "armServo2"),
                hardwareMap.get(Servo.class, "wristServo"),
                hardwareMap.get(Servo.class, "innerPixelServo"),
                telemetry
        );

        m_armSubsystem = new ArmSubsystem(
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armLeft"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armRight"),
                telemetry
        );

        m_droneSubsystem = new DroneSubsystem(
                hardwareMap.get(Servo.class, "droneServo")
        );

        /* DEFAULT COMMANDS */
        //these run constantly in the background
        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x
        ));

        m_armSubsystem.setDefaultCommand(new ArmCommand(
                m_armSubsystem,
                m_clawSubsystem,
                () -> mechGamepad.getLeftY(),
                () -> -mechGamepad.getRightY(),
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> driverGamepad.getGamepadButton(GamepadKeys.Button.A).get(),
                () -> mechGamepad.getGamepadButton(GamepadKeys.Button.A).get(),
                () -> driverGamepad.getGamepadButton(GamepadKeys.Button.Y).get(),
                () -> mechGamepad.getGamepadButton(GamepadKeys.Button.Y).get(),
                () -> driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get(),
                () -> mechGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get(),
                () -> mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).get(),
                () -> mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get()
        ));

        /* DRIVER CONTROLS */
        //Instant Command runs a subsytem method as if it were a full command (saves time)
        driverGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> m_driveSubsystem.zeroHeading()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> m_driveSubsystem.setMaxSpeed(Constants.DRIVE_GRAB_SPEED)))
                .whenReleased(new InstantCommand(() -> m_driveSubsystem.setMaxSpeed(Constants.DRIVE_DEFAULT_SPEED)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> m_clawSubsystem.setClawOpen()));
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new InstantCommand(() -> m_clawSubsystem.setClawClosed()));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> m_armSubsystem.extendSlide()));
        //driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> m_armSubsystem.retractSlide()));
        /* MECH CONTROLS */
        mechGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new ResetArmCommand(m_armSubsystem));
        mechGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> m_clawSubsystem.setClawOpen()));
        mechGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new InstantCommand(() -> m_clawSubsystem.setClawClosed()));
        mechGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> m_droneSubsystem.launch()));
    }
}
