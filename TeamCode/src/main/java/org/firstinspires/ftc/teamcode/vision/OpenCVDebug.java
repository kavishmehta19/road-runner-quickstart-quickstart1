package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="OpenCVDebug")
public class OpenCVDebug extends LinearOpMode {

    SampleMecanumDrive drive;

    OpenCvCamera phoneCam;
    WebcamName webcamName;

    CenterStagePipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new CenterStagePipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            telemetry.update();
        }
    }

    public static class CenterStagePipeline extends OpenCvPipeline {
        public enum Position {
            LEFT,
            CENTER,
            RIGHT
        }

        static final Scalar blue = new Scalar(0, 0, 255);
        static final Scalar red = new Scalar(255, 0, 0);
        static final Scalar black = new Scalar(0, 0, 0);
        static final Scalar white = new Scalar(255, 255, 255);

        public static Point anchor1 = new Point(23, 90);
        public static Point anchor2 = new Point(150, 75);
        public static Point anchor3 = new Point(295, 88);

        public static Scalar avgs = new Scalar(0, 0, 0);

        static final int width = 10;
        static final int height = 10;

        Point point1TL = new Point(
                anchor1.x,
                anchor1.y);
        Point point1BR = new Point(
                anchor1.x + width,
                anchor1.y + height);

        Point point2TL = new Point(
                anchor2.x,
                anchor2.y);
        Point point2BR = new Point(
                anchor2.x + width,
                anchor2.y + height);

        Point point3TL = new Point(
                anchor3.x,
                anchor3.y);
        Point point3BR = new Point(
                anchor3.x + width,
                anchor3.y + height);

        Mat region1;
        Mat region2;
        Mat region3;

        Mat YCrCb = new Mat();

        int avg1, avg2, avg3;

        public volatile Position position = Position.RIGHT;

        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCr(input);

            Mat ycrcbMat = new Mat();
            Mat binaryMat = new Mat();
            Mat maskedInputMat = new Mat();
            maskedInputMat.release();

            region1 = YCrCb.submat(new Rect(point1TL, point1BR));
            region2 = YCrCb.submat(new Rect(point2TL, point2BR));
            region3 = YCrCb.submat(new Rect(point3TL, point3BR));

             avg1 = Math.max((int) Core.mean(region1).val[1], (int) Core.mean(region1).val[2]);
             avg2 = Math.max((int) Core.mean(region2).val[1], (int) Core.mean(region2).val[2]);
             avg3 = Math.max((int) Core.mean(region3).val[1], (int) Core.mean(region3).val[2]);

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    point1TL, // First point which defines the rectangle
                    point1BR, // Second point which defines the rectangle
                    black, // The color the rectangle is drawn in
                    3); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    point2TL, // First point which defines the rectangle
                    point2BR, // Second point which defines the rectangle
                    black, // The color the rectangle is drawn in
                    3); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    point3TL, // First point which defines the rectangle
                    point3BR, // Second point which defines the rectangle
                    black, // The color the rectangle is drawn in
                    3); // Thickness of the rectangle lines

            if (avg2 >= avg3 && avg2 >= avg1) {
                position = Position.CENTER; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        point2TL, // First point which defines the rectangle
                        point2BR, // Second point which defines the rectangle
                        red, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            } else if (avg1 >= avg2 && avg1 >= avg3) {
                position = Position.LEFT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        point1TL, // First point which defines the rectangle
                        point1BR, // Second point which defines the rectangle
                        blue, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            } else {
                position = Position.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        point3TL, // First point which defines the rectangle
                        point3BR, // Second point which defines the rectangle
                        white, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            return input;
        }

        public String getAnalysis(){
            return "Box 1: " + avg1 + " Box 2: " + avg2 + " Box 3: " + avg3;
        }

    }

}
