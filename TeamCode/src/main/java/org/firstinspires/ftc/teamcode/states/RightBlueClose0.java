package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name="Right Blue Close Auton 2+0")

public class RightBlueClose0 extends LinearOpMode {

    SampleMecanumDrive drive;//drivetrain

    DcMotor liftL;//lift motors
    DcMotor liftR;

    Servo blocker;
    Servo blockerInner;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    OpenCvCamera phoneCam;//camera
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;//opencv pipeline

    OpenCVDebug.CenterStagePipeline.Position position;

    CRServo roller;

    int numCycles = 0;

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

        webcamName = hardwareMap.get(WebcamName.class, "webcam");//opencv
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

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startBR)//trajectory sequence
                .lineToLinearHeading(Constants.purpleCenterOffsetBR)
                .lineToLinearHeading(Constants.purpleCenterBR)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleCenterOffsetBR)
                .lineToLinearHeading(Constants.purpleCloseOffsetBR)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startBR)
                .lineToLinearHeading(Constants.purpleLeftOffsetBR)
                .lineToLinearHeading(Constants.purpleLeftBR)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleLeftOffsetBR)
                .lineToLinearHeading(Constants.purpleCloseOffsetBR)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startBR)
                .lineToLinearHeading(Constants.purpleRightCloseOffsetBR)
                .lineToLinearHeading(Constants.purpleRightCloseBR)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleRightCloseOffsetBR)
                .lineToLinearHeading(Constants.purpleCloseOffsetBR)
                .build();

        TrajectorySequence intake = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
                .addTemporalMarker(0.01,()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                    dropdown.setPosition(Constants.dropdownIntakeStart);
                    Constants.setIntake(1);
                })
                .lineToLinearHeading(Constants.closeIntakeOffsetBR)
                .lineToLinearHeading(Constants.closeIntakeBR)
                .lineToLinearHeading(Constants.closeIntakeOffsetBR)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseBR)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.yellowCloseCenterBR)
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(Constants.yellowCloseCenterBR)
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
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.closeParkOffsetBR)
                .lineToLinearHeading(Constants.closeParkBR)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseBR)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.yellowCloseLeftBR)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.yellowCloseLeftBR)
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
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.closeParkOffsetBR)
                .lineToLinearHeading(Constants.closeParkBR)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
                .addTemporalMarker(2.5, () -> {
                    Constants.setLift(Constants.liftTargetFar, 1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                })
                .addTemporalMarker(3.5, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.prepareDropCloseBR)
                .addDisplacementMarker(() -> {
                    Constants.setIntake(0);
                })
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.yellowCloseRightBR)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.yellowCloseRightBR)
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
                .lineToLinearHeading(Constants.yellowResetBR)
                .lineToLinearHeading(Constants.closeParkOffsetBR)
                .lineToLinearHeading(Constants.closeParkBR)
                .build();

        while (!isStarted()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(Constants.startBR);

            position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionAuton);
            sleep(800);

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

