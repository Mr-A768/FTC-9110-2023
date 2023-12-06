package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autoLibs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autoLibs.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous(group = "test")
//@Disabled
public class RF_AutoTester extends LinearOpMode {

    private ClawSubsystem m_clawSubsystem;
    private ArmSubsystem m_armSubsystem;

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

        m_armSubsystem = new ArmSubsystem(
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armLeft"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "armRight"),
                telemetry
        );

        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeqL = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(m_armSubsystem.getArmAngle());
                    m_armSubsystem.setSlideLengthByTick(0);
                    m_clawSubsystem.setWristUp();
                })
                .lineToConstantHeading(new Vector2d(-36, -48))
                .splineToSplineHeading(new Pose2d(-47, -34.5, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(-43, -34))
                .splineToConstantHeading(new Vector2d(-30, -12), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(48, -29), Math.toRadians(0))
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(46, -29))
                .splineToSplineHeading(new Pose2d(24, -12, Math.toRadians(180.1)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -12), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24, -12))
                .splineToSplineHeading(new Pose2d(48, -36, Math.toRadians(.2)), Math.toRadians(0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(42, -36, Math.toRadians(90)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeqL);
    }
}

