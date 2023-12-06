package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private DoubleSupplier m_translationXSupplier, m_translationYSupplier, m_rotationSupplier;

    public DriveCommand(DriveSubsystem driveSubsystem,
                        DoubleSupplier translationXSupplier,
                        DoubleSupplier translationYSupplier,
                        DoubleSupplier rotationSupplier) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        double translationX = m_translationXSupplier.getAsDouble() * m_driveSubsystem.maxSpeed;
        // Filter out values less than 0.1
        if (Math.abs(translationX) < 0.05){
            translationX = 0;
        }
        // Square the forward stick but keep the sign
        translationX = Math.copySign(Math.pow(translationX, 3.0), translationX);

        double translationY = m_translationYSupplier.getAsDouble() * m_driveSubsystem.maxSpeed;
        // Filter out values less than 0.1
        if (Math.abs(translationY) < 0.05){
            translationY = 0;
        }
        // Square the strafe stick but keep the sign
        translationY = Math.copySign(Math.pow(translationY, 3.0), translationY);

        double rotation = m_rotationSupplier.getAsDouble() * m_driveSubsystem.maxSpeed;
        // Filter out values less than 0.15
        if (Math.abs(rotation) < 0.05){
            rotation = 0;
        }
        // Square the rotation stick but keep the sign
        rotation = Math.copySign(Math.pow(rotation, 3.0), rotation);


        m_driveSubsystem.drive(translationX, translationY, rotation, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}