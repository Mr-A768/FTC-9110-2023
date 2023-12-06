package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ArmToBoardPosCommand extends CommandBase {
    double targetBoardPos;
    double armAngle;

    ArmSubsystem m_armSubsystem;
    ClawSubsystem m_clawSubsystem;

    public ArmToBoardPosCommand(double targetBoardPos,
                                ArmSubsystem armSubsystem,
                                ClawSubsystem clawSubsystem) {
        this.targetBoardPos = targetBoardPos;
        this.m_armSubsystem = armSubsystem;
        this.m_clawSubsystem = clawSubsystem;

        addRequirements(m_armSubsystem, m_clawSubsystem);
    }

    @Override
    public void initialize(){
        m_clawSubsystem.setApparentClawAngle(Math.toRadians(60));
        armAngle = Math.atan((Math.sqrt(3)/2) * targetBoardPos - Constants.ARM_PIVOT_HEIGHT);
        m_armSubsystem.setArmAngle(armAngle);
        m_armSubsystem.setSlideLengthByAngle(armAngle);
        m_clawSubsystem.setWristAngle(m_clawSubsystem.apparentClawAngle - m_armSubsystem.getArmAngle());
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
