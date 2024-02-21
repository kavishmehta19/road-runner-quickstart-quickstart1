package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Left Red Auton Purple Park")

public class LeftRedAutonPurplePark extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;

    Servo blocker;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;

    CRServo roller;

    Pose2d startPose = Constants.startPoseRL;
    Pose2d purplepixelcenterRL = Constants.purplepixelcenterRL;

    Pose2d purplepixelleftRL = Constants.purplepixelleftRL;
    Pose2d yellowpixelcenterRL = Constants.yellowpixelcenterRL;
    Pose2d parkL = Constants.parkL;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;

        blocker = Constants.blocker;
        roller = Constants.roller;
        tiltL = Constants.tiltL;
        tiltR = Constants.tiltR;
        airplane = Constants.airplane;
        dropdown = Constants.dropdown;

        blocker.setPosition(Constants.blockerClosedPosition);
        dropdown.setPosition(Constants.dropdownPositionUp);

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new OpenCVDebug.CenterStagePipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(purplepixelcenterRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(400);
                })
                .lineToLinearHeading(Constants.purplepixelRLoffset)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.startOffsetRL)
                .lineToLinearHeading(Constants.purplepixelleftRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.whitepixelRed2offset)
                .lineToLinearHeading(Constants.purplepixelRLoffset)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.purplepixelrightRL)
                .lineToLinearHeading(Constants.purplepixelrightRLoffset)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.purplepixelrightRL)
                .lineToLinearHeading(Constants.purplepixelRLoffset)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purplepixelRLoffset)
                .lineToLinearHeading(Constants.whitepixelRed2)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purplepixelRLoffset)
                .lineToLinearHeading(Constants.whitePixelRed2Halfway)
                .lineToLinearHeading(Constants.whitepixelRed2)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purplepixelRLoffset)
                .lineToLinearHeading(Constants.whitepixelRed2)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(startPose);

            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionStart);
            sleep(1000);

            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft);
                drive.followTrajectorySequence(yellowLeft);
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
            }
            else {
                drive.followTrajectorySequence(purpleRight);
                dropdown.setPosition(Constants.dropdownPositionUp);
                sleep(300);
                drive.followTrajectorySequence(yellowRight);
            }
            sleep(2000);

            break;

        }
    }



}

