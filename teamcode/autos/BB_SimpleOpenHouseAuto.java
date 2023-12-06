package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autoLibs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autoLibs.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous(group = "drive")
@Disabled
public class BB_SimpleOpenHouseAuto extends LinearOpMode {

    private ClawSubsystem m_clawSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        m_clawSubsystem = new ClawSubsystem(
                hardwareMap.get(Servo.class, "armServo"),
                hardwareMap.get(Servo.class, "armServo2"),
                hardwareMap.get(Servo.class, "wristServo"),
                hardwareMap.get(Servo.class, "innerPixelServo"),
                telemetry
        );

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawClosed();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(23, 48))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

