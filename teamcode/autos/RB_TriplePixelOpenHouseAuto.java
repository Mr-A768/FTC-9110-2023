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
public class RB_TriplePixelOpenHouseAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                    m_clawSubsystem.setClawClosed();
                })
                .addTemporalMarker(1.5, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .lineToConstantHeading(new Vector2d(52, -60))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                    m_clawSubsystem.setClawOpen();
                })
                .lineToLinearHeading(new Pose2d(12, -37.5, Math.toRadians(90)))
                .addTemporalMarker(5.5, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addTemporalMarker(6, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                })
                .addTemporalMarker(8, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .lineToConstantHeading(new Vector2d(43, -30))
                .addTemporalMarker(8.64, () -> {
                    m_clawSubsystem.setWristUp();
                    m_clawSubsystem.setClawOpen();
                })
                .splineToLinearHeading(new Pose2d(-2.5, -13.5, Math.toRadians(90)), Math.toRadians(180))
                .addTemporalMarker(11.5, () -> {
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
                .addTemporalMarker(18, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                    m_clawSubsystem.setClawOpen();
                })
                .lineToLinearHeading(new Pose2d(-11, 0, Math.toRadians(180)))
                .addTemporalMarker(21.7, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addTemporalMarker(22.2, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setWristUp();
                })
                .lineToLinearHeading(new Pose2d(52, -13, Math.toRadians(0)))
                .addTemporalMarker(27, () -> {
                    m_clawSubsystem.setWristDown();
                })
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                })
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

