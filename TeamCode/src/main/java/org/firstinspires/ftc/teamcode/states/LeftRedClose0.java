package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Left Red Close Auton 2+0")

public class LeftRedClose0 extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;

    Servo blocker;
    Servo blockerInner;

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
        blockerInner = Constants.blockerInner;
        roller = Constants.roller;
        tiltL = Constants.tiltL;
        tiltR = Constants.tiltR;
        airplane = Constants.airplane;
        dropdown = Constants.dropdown;

        blocker.setPosition(Constants.blockerClosedPosition);
        blockerInner.setPosition(Constants.blockerInnerOpenPosition);
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
                .lineToLinearHeading(Constants.purpleCenterOffsetRL)
                .lineToLinearHeading(Constants.purpleCenterRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleCenterOffsetRL)
                .lineToLinearHeading(Constants.purpleCloseOffsetRL)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startRL)
                .lineToLinearHeading(Constants.purpleLeftCloseOffsetRL)
                .lineToLinearHeading(Constants.purpleLeftCloseRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleLeftCloseOffsetRL)
                .lineToLinearHeading(Constants.purpleCloseOffsetRL)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startRL)
                .lineToLinearHeading(Constants.purpleRightOffsetRL)
                .lineToLinearHeading(Constants.purpleRightRL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleRightOffsetRL)
                .lineToLinearHeading(Constants.purpleCloseOffsetRL)
                .build();

        TrajectorySequence intake = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetRL)
                .addTemporalMarker(0.01,()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                    dropdown.setPosition(Constants.dropdownIntakeStart);
                    Constants.setIntake(1);
                })
                .lineToLinearHeading(Constants.closeIntakeOffsetRL)
                .lineToLinearHeading(Constants.closeIntakeRL)
                .lineToLinearHeading(Constants.closeIntakeOffsetRL)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.yellowCenterRL)
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(Constants.yellowCenterRL)
                .addTemporalMarker(0.1,()->{
                    Constants.setLift(Constants.liftTargetAuton2, 1);

                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .addTemporalMarker(0.9, () -> {
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.farParkOffsetRL)
                .lineToLinearHeading(Constants.farParkRL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.yellowLeftRL)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.yellowLeftRL)
                .addTemporalMarker(0.1,()->{
                    Constants.setLift(Constants.liftTargetAuton2, 1);

                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .addTemporalMarker(0.9, () -> {
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.farParkOffsetRL)
                .lineToLinearHeading(Constants.farParkRL)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetRL)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseRL)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowOffsetRL)
                .lineToLinearHeading(Constants.yellowRightRL)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.yellowRightRL)
                .addTemporalMarker(0.1,()->{
                    Constants.setLift(Constants.liftTargetAuton2, 1);

                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                })
                .addTemporalMarker(0.9, () -> {
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
                drive.followTrajectorySequence(yellowLeft);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(600);
                drive.followTrajectorySequence(parkLeft);
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(600);
                drive.followTrajectorySequence(parkCenter);
            }
            else {
                drive.followTrajectorySequence(purpleRight);
                drive.followTrajectorySequence(yellowRight);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(600);
                drive.followTrajectorySequence(parkRight);
            }

            break;

        }
    }



}

