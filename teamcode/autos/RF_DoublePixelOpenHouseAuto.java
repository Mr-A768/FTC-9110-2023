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
public class RF_DoublePixelOpenHouseAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                    m_clawSubsystem.setClawClosed();
                })
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)))
                .addTemporalMarker(6, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .lineToConstantHeading(new Vector2d(52, -12))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(-2.5, -13, Math.toRadians(90)))
                .addTemporalMarker(11.1, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addTemporalMarker(12, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(52, -13, Math.toRadians(0)))
                .addTemporalMarker(16, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(-10.5, 2, Math.toRadians(180)))
                .addTemporalMarker(21.3, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addTemporalMarker(22, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(52.5, -13, Math.toRadians(0)))
                .addTemporalMarker(27, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                    m_clawSubsystem.setWristUp();
                })
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

