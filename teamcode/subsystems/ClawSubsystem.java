package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class ClawSubsystem extends SubsystemBase {

    private Servo sClawLeft, sClawRight, sWrist, sInnerPixel;

    private Telemetry telemetry;

    private boolean clawIsOpen = true;
    private boolean wristIsUp = true;
    public boolean scoreMode = false;
    public double apparentClawAngle = Math.toRadians(60);

    public ClawSubsystem(Servo armServo,
                         Servo armServo2,
                         Servo wristServo,
                         Servo innerPixelServo,
                         Telemetry telemetry) {
        this.sClawLeft = armServo;
        this.sClawRight = armServo2;
        this.sWrist = wristServo;
        this.sInnerPixel = innerPixelServo;
        this.telemetry = telemetry;

        //sClawLeft.scaleRange(clawMin, clawMax);
        //sClawRight.scaleRange(1.0 - clawMin, 1.0 - clawMax);

        sWrist.setDirection(Servo.Direction.REVERSE);
        sWrist.scaleRange(Constants.WRIST_MIN, Constants.WRIST_MAX);

        sInnerPixel.scaleRange(0.15, 1);
        //sets positions at the start so the toggles are easier to use
        setClawClosed();
        setWristUp();

        sInnerPixel.setPosition(1);
    }

    @Override
    public void periodic() {
    }

    public void setClawOpen(){
        sClawLeft.setPosition(Constants.CLAW_MAX);
        sClawRight.setPosition(1-Constants.CLAW_MAX);
        clawIsOpen = true;
    }

    public void setClawClosed(){
        sClawLeft.setPosition(Constants.CLAW_MIN);
        sClawRight.setPosition(1-Constants.CLAW_MIN);
        clawIsOpen = false;
    }

    public void setWristUp(){
        sWrist.setPosition(1);//0.739
        wristIsUp = true;
    }

    public void setWristDown(){
        sWrist.setPosition(0);//0.239
        wristIsUp = false;
    }

    public void toggleClaw(){
        if (clawIsOpen == true) {
            setClawClosed();
        } else {
            setClawOpen();
        }
    }

    public void toggleWrist(){
        if (wristIsUp == true) {
            setWristDown();
        } else {
            setWristUp();
        }
    }

    public void stepWrist(boolean up){
        if (up && sWrist.getPosition() <= Constants.WRIST_MAX){
            sWrist.setPosition(sWrist.getPosition() + .05);
        } else if (!up && sWrist.getPosition() >= Constants.WRIST_MIN){
            sWrist.setPosition(sWrist.getPosition() - .05);
        }
    }

    //Wrist should be able to move to 125 degrees off of the arm's angle in either direction
    public void setWristAngle(double angle){
        angle = angle / Math.PI;    //Translates angle to servo position
        sWrist.setPosition(angle);
    }

    public void setScoreMode(boolean modeToSet){
        scoreMode = modeToSet;
    }

    public void setApparentClawAngle(double angle){
        apparentClawAngle = angle;
    }

    public void setWristStow(){
        setWristAngle(Math.toRadians(90) - Constants.ARM_DRIVEMODE_ANGLE);
    }

    public void dropPixel(){
        sInnerPixel.setPosition(0);
    }

    public void resetPixelDropper(){
        sInnerPixel.setPosition(1);
    }

}

