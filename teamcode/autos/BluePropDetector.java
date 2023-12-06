package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
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
public class BluePropDetector extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pipeline = new rectangle_thresholder_pipeline();

        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //webcam.openCameraDevice();//open camera
        webcam.setPipeline(new rectangle_thresholder_pipeline());//different stages
        //webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
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

        waitForStart();
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
            if (w1 > w2 + 100 && w1  > w3 + 100) {
                location = "left";
            } else if (w2 > w1 + 100 && w2 > w3 + 100) {
                location = "middle";
            } else if (w3 > w1 + 100 && w3 > w2 + 100){
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