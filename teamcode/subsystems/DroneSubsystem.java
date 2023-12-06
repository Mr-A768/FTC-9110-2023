package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubsystem extends SubsystemBase {
    Servo sDroneLauncher;

    public DroneSubsystem(Servo sDroneLauncher) {
        this.sDroneLauncher = sDroneLauncher;
        sDroneLauncher.setDirection(Servo.Direction.REVERSE);
        sDroneLauncher.scaleRange(.5, .7);
        sDroneLauncher.setPosition(0);
    }

    @Override
    public void periodic() {
    }

    public void launch() {
        sDroneLauncher.setPosition(1);
    }

}

