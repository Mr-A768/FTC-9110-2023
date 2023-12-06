package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
    ArmSubsystem m_armSubsystem;
    ClawSubsystem m_clawSubsystem;

    DoubleSupplier m_rightY, m_leftY, m_leftTrigger, m_rightTrigger;
    Double setArmSpeed, setSlideSpeed;
    Boolean armHasBeenMoved = false, armHasBeenMovedSlowly = false, slideHasBeenMoved = false;
    BooleanSupplier d_aButton, m_aButton, d_leftBumper, d_yButton, m_yButton, m_leftBumper, m_Up, m_Down;

    public ArmCommand(ArmSubsystem armSubsystem,
                      ClawSubsystem clawSubsystem,
                      DoubleSupplier m_leftY,
                      DoubleSupplier m_rightY,
                      DoubleSupplier m_leftTrigger,
                      DoubleSupplier m_rightTrigger,
                      BooleanSupplier d_aButton,
                      BooleanSupplier m_aButton,
                      BooleanSupplier d_yButton,
                      BooleanSupplier m_yButton,
                      BooleanSupplier d_leftBumper,
                      BooleanSupplier m_leftBumper,
                      BooleanSupplier m_Up,
                      BooleanSupplier m_Down) {
        this.m_leftY = m_leftY;
        this.m_rightY = m_rightY;
        this.m_leftTrigger = m_leftTrigger;
        this.m_rightTrigger = m_rightTrigger;
        this.m_armSubsystem = armSubsystem;
        this.m_clawSubsystem = clawSubsystem;
        this.d_aButton = d_aButton;
        this.m_aButton = m_aButton;
        this.d_yButton = d_yButton;
        this.m_yButton = m_yButton;
        this.d_leftBumper = d_leftBumper;
        this.m_leftBumper = m_leftBumper;
        this.m_Up = m_Up;
        this.m_Down = m_Down;

        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize(){
        m_clawSubsystem.setApparentClawAngle(Math.toRadians(60));
        m_armSubsystem.setSlideLengthByTick(0);
    }

    @Override
    public void execute(){
        /* Control Config */
        setArmSpeed = m_leftY.getAsDouble();
        //setArmSpeed = m_leftTrigger.getAsDouble() + -m_rightTrigger.getAsDouble();
        setSlideSpeed = m_rightY.getAsDouble();

        /* Controller Deadband */
        if (Math.abs(setArmSpeed) > 0.05){
            setArmSpeed = setArmSpeed * Constants.ARM_MAX_ANGULAR_SPEED;
        } else {
            setArmSpeed = 0.0;
        }
        if (Math.abs(setSlideSpeed) > 0.05){
            setSlideSpeed = setSlideSpeed * Constants.SLIDE_MAX_SPEED * .85;
        } else {
            setSlideSpeed = 0.0;
        }

        /* Angle Limits */
        if ((m_armSubsystem.getArmAngle() < Constants.ARM_MIN_ANGLE && setArmSpeed < 0)
                || (m_armSubsystem.getArmAngle() > Constants.ARM_MAX_ANGLE && setArmSpeed > 0)){
            setArmSpeed = 0.0;
        }

        /* Slide Limits */
        if ((m_armSubsystem.getSlideTick() < Constants.SLIDE_MIN_TICK && setSlideSpeed < 0)
                || (m_armSubsystem.getSlideTick() > Constants.SLIDE_MAX_TICK && setSlideSpeed > 0)){
            setSlideSpeed = 0.0;
        }

        /* Mode Setter */
        if (d_aButton.getAsBoolean()){
            m_clawSubsystem.setScoreMode(false);
            m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_ANGLE);
            m_armSubsystem.setSlideLengthByTick(Constants.SLIDE_MIN_TICK);
        }
        if (m_aButton.getAsBoolean() && !d_aButton.getAsBoolean()){
            m_clawSubsystem.setScoreMode(true);
            m_armSubsystem.setArmAngle(Constants.ARM_SCOREMODE_ANGLE);
            armHasBeenMoved = false;
        }

        /* Mode Runner */
        if (m_clawSubsystem.scoreMode){ // SCORE MODE //
            if (setArmSpeed != 0){
                m_armSubsystem.moveArm(setArmSpeed);
                armHasBeenMoved = true;
            } else if (setArmSpeed == 0 && armHasBeenMoved){
                if (m_armSubsystem.getArmAngle() > Constants.ARM_MAX_ANGLE){
                    m_armSubsystem.setArmAngle(Constants.ARM_MAX_ANGLE);
                } else if (m_armSubsystem.getArmAngle() < Constants.ARM_MIN_ANGLE){
                    m_armSubsystem.setArmAngle(Constants.ARM_MIN_ANGLE);
                } else {
                    m_armSubsystem.setArmAngle(m_armSubsystem.getArmAngle());
                }
                armHasBeenMoved = false;
            }
            if (m_yButton.getAsBoolean()){
                m_armSubsystem.setArmAngle(Constants.ARM_MAX_ANGLE);
            }
            if (m_leftBumper.getAsBoolean()) {
                m_armSubsystem.setSlideLengthByAngle(m_armSubsystem.getArmAngle());
            }else {
                if (setSlideSpeed != 0){
                    m_armSubsystem.runSlide(setSlideSpeed);
                    slideHasBeenMoved = true;
                } else if (setSlideSpeed == 0 && slideHasBeenMoved){
                    if (m_armSubsystem.getSlideTick() > Constants.SLIDE_MAX_TICK){
                        m_armSubsystem.setSlideLengthByTick(Constants.SLIDE_MAX_TICK);
                    } else if (m_armSubsystem.getSlideTick() < Constants.SLIDE_MIN_TICK){
                        m_armSubsystem.setArmAngle(Constants.SLIDE_MIN_TICK);
                    } else {
                        m_armSubsystem.setSlideLengthByTick(m_armSubsystem.getSlideTick());
                    }
                    slideHasBeenMoved = false;
                }
            }
            m_clawSubsystem.setWristAngle(m_clawSubsystem.apparentClawAngle - m_armSubsystem.getArmAngle());
        } else { // DRIVE MODE //
            if (d_yButton.getAsBoolean()){
                m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_STACK_ANGLE);
            }
            if (d_leftBumper.getAsBoolean()){
               m_clawSubsystem.setWristAngle(-m_armSubsystem.getArmAngle());
            } else {
                m_clawSubsystem.setWristUp();
            }
        }

        /* Fine Arm Angle Controls */
        if (m_Up.getAsBoolean() && !m_Down.getAsBoolean() && setArmSpeed == 0){
            m_armSubsystem.moveArm(Constants.ARM_LOW_ANGULAR_SPEED);
            armHasBeenMovedSlowly = true;
        } else if (!m_Up.getAsBoolean() && m_Down.getAsBoolean() && setArmSpeed == 0) {
            m_armSubsystem.moveArm(-Constants.ARM_LOW_ANGULAR_SPEED);
            armHasBeenMovedSlowly = true;
        } else if (!m_Up.getAsBoolean() && !m_Down.getAsBoolean() && setArmSpeed == 0 && armHasBeenMovedSlowly){
            m_armSubsystem.setArmAngle(m_armSubsystem.getArmAngle());
            armHasBeenMovedSlowly = false;
        }
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