package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.autoLibs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autoLibs.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class BF_BingBingBing extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard;
    rectangle_thresholder_pipeline pipeline;
    OpenCvCamera webcam;

    private final int rows = 1280;
    private final int cols = 720;

    private static String location = "nothing"; // output

    // Rectangle regions to be scanned
    private static Point topLeft1 = new Point(0, 450), bottomRight1 = new Point(150, 600);
    private static Point topLeft2 = new Point(640 - 100, 450), bottomRight2 = new Point(640 + 100, 600);
    private static Point topLeft3 = new Point(1280 - 150, 450), bottomRight3 = new Point(1280, 600);

    private ClawSubsystem m_clawSubsystem;
    private ArmSubsystem m_armSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new rectangle_thresholder_pipeline();

        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        webcam.setPipeline(new rectangle_thresholder_pipeline());//different stages
        //width, height
        //width = height in this case, because camera is in portrait mode.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode) {
                //This will be called if the camera could not be opened
            }
        });

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

        Pose2d startPose = new Pose2d(-36, 61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeqR = drive.trajectorySequenceBuilder(startPose) //TODO:
                .addDisplacementMarker(() -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngleByTick(-Constants.ARM_INIT_TICK);
                })
                .lineToConstantHeading(new Vector2d(-36, 48))
                .splineToSplineHeading(new Pose2d(-40, 34.5, Math.toRadians(0)), Math.toRadians(180.1))
                .lineToConstantHeading(new Vector2d(-47, 34.5))
                .addTemporalMarker(2.75, () -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngle(Math.toRadians(0));
                })
                .addTemporalMarker(3, () -> {
                    m_clawSubsystem.dropPixel();
                })
                .waitSeconds(.25)
                .addTemporalMarker(3.75, () -> {
                    m_clawSubsystem.resetPixelDropper();
                })
                .addTemporalMarker(4.2, () -> {
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_ANGLE);
                })
                .lineToConstantHeading(new Vector2d(-46, 34.5))
                .splineToConstantHeading(new Vector2d(-29.5, 12), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, 12))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(2));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(2));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(2));
                })
                .splineToConstantHeading(new Vector2d(51.5, 30.5), Math.toRadians(0))
                .addTemporalMarker(9.5, () -> {
                    m_clawSubsystem.setClawOpen();
                })
                .waitSeconds(2)
                .addTemporalMarker(12.5, () -> {
                    m_armSubsystem.setSlideLengthByTick(0);
                    m_clawSubsystem.setClawClosed();
                })
                .lineToConstantHeading(new Vector2d(46, 44))
                .addTemporalMarker(12, () -> {
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_STACK_ANGLE);
                    m_clawSubsystem.setWristAngle(Math.toRadians(0) - Constants.ARM_DRIVEMODE_STACK_ANGLE);
                })
                .splineToSplineHeading(new Pose2d(24, 14, Math.toRadians(179.9)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                })
                .splineToConstantHeading(new Vector2d(-57, 14), Math.toRadians(180))
                .addTemporalMarker(16.5, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .addTemporalMarker(18, () -> {
                    m_clawSubsystem.setWristUp();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24, 14))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(9));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(9));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(9));
                })
                .splineToSplineHeading(new Pose2d(52, 36, Math.toRadians(359.7)), Math.toRadians(0))
                .addTemporalMarker(23, () -> {
                    m_clawSubsystem.setClawOpen();
                    m_armSubsystem.setSlideLength(0);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(25, () -> {
                    m_clawSubsystem.setClawClosed();
                    m_clawSubsystem.setWristUp();
                    m_armSubsystem.setArmAngleByTick(Constants.ARM_INIT_TICK);
                })
                .lineToLinearHeading(new Pose2d(42, 36, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeqM = drive.trajectorySequenceBuilder(startPose) //TODO:
                .addDisplacementMarker(() -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngleByTick(-Constants.ARM_INIT_TICK);
                })
                .lineToConstantHeading(new Vector2d(-36, 28.5))
                .addTemporalMarker(3, () -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngle(Math.toRadians(0));
                })
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.dropPixel();
                })
                .waitSeconds(.25)
                .addTemporalMarker(3.5, () -> {
                    m_clawSubsystem.resetPixelDropper();
                })
                .addTemporalMarker(4.2, () -> {
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_ANGLE);
                })
                .lineToConstantHeading(new Vector2d(-36, 9))
                .waitSeconds(.01)
                .splineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, 12))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(2));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(2));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(2));
                })
                .splineToConstantHeading(new Vector2d(49, 39.5), Math.toRadians(0))
                .addTemporalMarker(10, () -> {
                    m_clawSubsystem.setClawOpen();
                })
                .waitSeconds(2)
                .addTemporalMarker(12.5, () -> {
                    m_armSubsystem.setSlideLengthByTick(0);
                    m_clawSubsystem.setClawClosed();
                })
                .lineToConstantHeading(new Vector2d(46, 38))
                .addTemporalMarker(13, () -> {
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_STACK_ANGLE);
                    m_clawSubsystem.setWristAngle(Math.toRadians(0) - Constants.ARM_DRIVEMODE_STACK_ANGLE);
                })
                .splineToSplineHeading(new Pose2d(24, 14, Math.toRadians(179.9)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                })
                .splineToConstantHeading(new Vector2d(-56, 14), Math.toRadians(180))
                .addTemporalMarker(17.5, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .addTemporalMarker(19, () -> {
                    m_clawSubsystem.setWristUp();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24, 14))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(9));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(9));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(9));
                })
                .splineToSplineHeading(new Pose2d(50, 36, Math.toRadians(359.7)), Math.toRadians(0))
                .addTemporalMarker(25, () -> {
                    m_clawSubsystem.setClawOpen();
                    m_armSubsystem.setSlideLength(0);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(26, () -> {
                    m_clawSubsystem.setClawClosed();
                    m_clawSubsystem.setWristUp();
                    m_armSubsystem.setArmAngleByTick(Constants.ARM_INIT_TICK);
                })
                .lineToLinearHeading(new Pose2d(42, 36, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeqL = drive.trajectorySequenceBuilder(startPose) //TODO:
                .lineTo(new Vector2d(-36, 44))
                .addTemporalMarker(0.1, () -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngleByTick(-Constants.ARM_INIT_TICK);
                })
                .splineToConstantHeading(new Vector2d(-30, 37), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-22, 37))
                .addTemporalMarker(2.25, () -> {
                    m_clawSubsystem.dropPixel();
                })
                .addTemporalMarker(3, () -> {
                    m_armSubsystem.resetArmEncoders();
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_ANGLE);
                })
                .waitSeconds(.25)
                .addTemporalMarker(5, () -> {
                    m_clawSubsystem.resetPixelDropper();
                })
                .lineToConstantHeading(new Vector2d(-30, 37))
                .splineToConstantHeading(new Vector2d(-36, 27), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, 12))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(2));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(2));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(2));
                })
                .splineToConstantHeading(new Vector2d(51, 44), Math.toRadians(0))
                .addTemporalMarker(11, () -> {
                    m_clawSubsystem.setClawOpen();
                })
                .waitSeconds(2)
                .addTemporalMarker(12.5, () -> {
                    m_armSubsystem.setSlideLengthByTick(0);
                    m_clawSubsystem.setClawClosed();
                })
                .lineToConstantHeading(new Vector2d(46, 30))
                .addTemporalMarker(14, () -> {
                    m_armSubsystem.setArmAngle(Constants.ARM_DRIVEMODE_STACK_ANGLE);
                    m_clawSubsystem.setWristAngle(Math.toRadians(0) - Constants.ARM_DRIVEMODE_STACK_ANGLE);
                })
                .splineToSplineHeading(new Pose2d(24, 14, Math.toRadians(179.9)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    m_clawSubsystem.setClawOpen();
                })
                .splineToConstantHeading(new Vector2d(-55, 14), Math.toRadians(180))
                .addTemporalMarker(17.5, () -> {
                    m_clawSubsystem.setClawClosed();
                })
                .addTemporalMarker(21, () -> {
                    m_clawSubsystem.setWristUp();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24, 14))
                .addDisplacementMarker(() -> {
                    m_armSubsystem.setArmAngle(Math.toRadians(12));
                    m_armSubsystem.setSlideLengthByAngle(Math.toRadians(12));
                    m_clawSubsystem.setWristAngle(Math.toRadians(60) - Math.toRadians(12));
                })
                .splineToSplineHeading(new Pose2d(50, 36, Math.toRadians(359.7)), Math.toRadians(0))
                .addTemporalMarker(25, () -> {
                    m_clawSubsystem.setClawOpen();
                    m_armSubsystem.setSlideLength(0);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(27, () -> {
                    m_clawSubsystem.setClawClosed();
                    m_clawSubsystem.setWristUp();
                    m_armSubsystem.setArmAngleByTick(Constants.ARM_INIT_TICK);
                })
                .lineToLinearHeading(new Pose2d(42, 36, Math.toRadians(270)))
                .build();


        waitForStart();

        if (!isStopRequested()) {
            //drive.followTrajectorySequence(trajSeqL);
            if (pipeline.getLocation() == "left"){
                drive.followTrajectorySequence(trajSeqL);
            } else if (pipeline.getLocation() == "middle"){
                drive.followTrajectorySequence(trajSeqM);
            } else if (pipeline.getLocation() == "right"){
                drive.followTrajectorySequence(trajSeqR);
            } else {
                //drive.followTrajectorySequence(trajSeqE);
            }
        }

        if (isStopRequested()){
            webcam.closeCameraDevice();
            m_armSubsystem.stopArm();
        }

        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Location", pipeline.getLocation());
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

        }

    }
    static class rectangle_thresholder_pipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat(); // converted image
        private Mat binaryMat = new Mat(); // image analyzed after thresholding

        private Mat binaryMat2 = new Mat(); // image analyzed after thresholding

        double threshold = 50;

        @Override
        public Mat processFrame(Mat input) {
            // Convert from BGR to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, Constants.lowerB, Constants.upperB, binaryMat);


            // Scan both rectangle regions, keeping track of how many
            // pixels meet the threshold value, indicated by the color white
            // in the binary image
            double w1 = 0, w2 = 0, w3 = 0;
            // process the pixel value for each rectangle  (255 = W, 0 = B)
            for (int i = (int) topLeft1.y; i <= bottomRight1.y; i++) {
                for (int j = (int) topLeft1.x; j <= bottomRight1.x; j++) {
                    if (binaryMat.get(i, j) != null && binaryMat.get(i, j)[0] == 255) {
                        w1++;
                    }
                }
            }

            for (int i = (int) topLeft2.y; i <= bottomRight2.y; i++) {
                for (int j = (int) topLeft2.x; j <= bottomRight2.x; j++) {
                    if (binaryMat.get(i, j) != null && binaryMat.get(i, j)[0] == 255) {
                        w2++;
                    }
                }
            }

            for (int i = (int) topLeft3.y; i <= bottomRight3.y; i++) {
                for (int j = (int) topLeft3.x; j <= bottomRight3.x; j++) {
                    if (binaryMat.get(i, j) != null && binaryMat.get(i, j)[0] == 255) {
                        w3++;
                    }
                }
            }

            // Determine object location
            if (w1 > w2 + threshold && w1  > w3 + threshold) {
                location = "left";
            } else if (w2 > w1 + threshold && w2 > w3 + threshold) {
                location = "middle";
            } else if (w3 > w1 + threshold && w3 > w2 + threshold){
                location = "right";
            } else {
                location = "unclear";
            }

            Imgproc.rectangle(//1-3
                    binaryMat,
                    topLeft1,
                    bottomRight1,
                    new Scalar(200, 0, 0),
                    3);
            Imgproc.rectangle(//3-5
                    binaryMat,
                    topLeft2,
                    bottomRight2,
                    new Scalar(200, 0, 0),
                    3);
            Imgproc.rectangle(
                    binaryMat,
                    topLeft3,
                    bottomRight3,
                    new Scalar(200, 0, 0),
                    3
            );

            return binaryMat;
        }

        public String getLocation() {
            return location;
        }
    }
}