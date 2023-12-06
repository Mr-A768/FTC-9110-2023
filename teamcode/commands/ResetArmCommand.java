package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ResetArmCommand extends CommandBase {
    ArmSubsystem m_armSubsystem;

    public ResetArmCommand(ArmSubsystem armSubsystem) {
        this.m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize(){
        m_armSubsystem.resetArmEncoders();
        m_armSubsystem.setArmAngleByTick(-Constants.ARM_INIT_TICK);
    }

    @Override
    public void execute(){
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.resetArmEncoders();
        m_armSubsystem.setArmAngleByTick(0);
    }

    @Override
    public boolean isFinished() {
        if (m_armSubsystem.getArmTick() >= -Constants.ARM_INIT_TICK - 1 && m_armSubsystem.getArmTick() <= -Constants.ARM_INIT_TICK + 1){
            return true;
        } else {
            return false;
        }
    }
}
