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

@Autonomous(name="Left Red Auton 2+0")

public class LeftRedAuton extends LinearOpMode {

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

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startRL)
                .lineToLinearHeading(Constants.purpleCenterRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(400);
                })
                .lineToLinearHeading(Constants.purpleOffsetRL)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startRL)
                .lineToLinearHeading(Constants.startOffsetRL)
                .lineToLinearHeading(Constants.purpleLeftRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.purpleLeftOffsetRL)
                .lineToLinearHeading(Constants.purpleOffsetRL)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startRL)
                .lineToLinearHeading(Constants.purpleRightOffsetRL)
                .lineToLinearHeading(Constants.purpleRightRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.purpleRightOffsetRL)
                .lineToLinearHeading(Constants.purpleOffsetRL)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetLow, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowCenterRL)
                .addDisplacementMarker(()->{
                    blocker.setPosition(Constants.blockerOpenPosition);
                    sleep(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    Constants.setLift(Constants.liftTargetMid, 1);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.farParkOffsetRL)
                .lineToLinearHeading(Constants.farParkRL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetLow, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowLeftRL)
                .addDisplacementMarker(()->{
                    blocker.setPosition(Constants.blockerOpenPosition);
                    sleep(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    Constants.setLift(Constants.liftTargetMid, 1);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.farParkOffsetRL)
                .lineToLinearHeading(Constants.farParkRL)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetLow, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowRightRL)
                .addDisplacementMarker(()->{
                    blocker.setPosition(Constants.blockerOpenPosition);
                    sleep(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    Constants.setLift(Constants.liftTargetMid, 1);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.farParkOffsetRL)
                .lineToLinearHeading(Constants.farParkRL)
                .build();

        while (!isStarted()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(Constants.startRL);

            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionAuton);
            sleep(1000);

            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft);
                sleep(6500);
                drive.followTrajectorySequence(yellowLeft);
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                sleep(6500);
                drive.followTrajectorySequence(yellowCenter);
            }
            else {
                drive.followTrajectorySequence(purpleRight);
                dropdown.setPosition(Constants.dropdownPositionUp);
                sleep(6500);
                drive.followTrajectorySequence(yellowRight);
            }
            sleep(2000);

            break;

        }
    }



}

