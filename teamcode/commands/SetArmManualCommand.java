package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class SetArmManualCommand extends CommandBase {
    double armAngle, slideLength, clawApparentAngle;

    ArmSubsystem m_armSubsystem;
    ClawSubsystem m_clawSubsystem;

    public SetArmManualCommand(double armAngle,
                               double slideLength,
                               double clawApparentAngle,
                               ArmSubsystem armSubsystem,
                               ClawSubsystem clawSubsystem) {
        this.armAngle = armAngle;
        this.slideLength = slideLength;
        this.clawApparentAngle = clawApparentAngle;
        this.m_armSubsystem = armSubsystem;
        this.m_clawSubsystem = clawSubsystem;

        addRequirements(m_armSubsystem, m_clawSubsystem);
    }

    @Override
    public void initialize(){
        m_armSubsystem.setArmAngle(armAngle);
        m_clawSubsystem.setWristAngle(Math.toRadians(clawApparentAngle) - m_armSubsystem.getArmAngle());
        m_armSubsystem.setSlideLength(slideLength);
    }

    @Override
    public void execute(){

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
