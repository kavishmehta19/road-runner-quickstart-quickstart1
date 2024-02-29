package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@Autonomous(name = "Right Red Auton 2+0")

public class RightRedAuton extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;
    Servo blocker;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    ColorSensor colorSensor;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;

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
        roller = Constants.roller;
        tiltL = Constants.tiltL;
        tiltR = Constants.tiltR;
        airplane = Constants.airplane;
        dropdown = Constants.dropdown;

        colorSensor = Constants.colorSensor;

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
            public void onError(int errorCode) {
            }
        });

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startRR)
                .lineToLinearHeading(Constants.purpleCenterRR)
                .addDisplacementMarker(() -> {
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.purpleCenterOffsetRR)
                .lineToLinearHeading(Constants.backdropCenterOffsetRR)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startRR)
                .lineToLinearHeading(Constants.purpleLeftOffsetRR)
                .lineToLinearHeading(Constants.purpleLeftRR)
                .addDisplacementMarker(() -> {
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startRR)
                .lineToLinearHeading(Constants.purpleRightRR)
                .addDisplacementMarker(() -> {
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleRightOffsetRR)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.backdropCenterOffsetRR)
                .addTemporalMarker(0.1, () -> {
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.backdropCenterRR)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleLeftOffsetRR)
                .addTemporalMarker(0.1, () -> {
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.backdropLeftRR)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleRightRR)
                .addTemporalMarker(0.1, () -> {
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.backdropRightRR)
                .build();


        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(Constants.backdropCenterRR)
                .addTemporalMarker(0.2, () -> {
                    Constants.setLift(Constants.liftTargetMid, 1);
                })
                .addTemporalMarker(0.4,()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0,1);
                })
                .lineToLinearHeading(Constants.midParkRR)
                .lineToLinearHeading(Constants.closeParkOffsetRR)
                .lineToLinearHeading(Constants.closeParkRR)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.backdropLeftRR)
                .addTemporalMarker(0.2, () -> {
                    Constants.setLift(Constants.liftTargetMid, 1);
                })
                .addTemporalMarker(0.4,()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0,1);
                })
                .lineToLinearHeading(Constants.midParkRR)
                .lineToLinearHeading(Constants.closeParkOffsetRR)
                .lineToLinearHeading(Constants.closeParkRR)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.backdropRightRR)
                .addTemporalMarker(0.2, () -> {
                    Constants.setLift(Constants.liftTargetMid, 1);
                })
                .addTemporalMarker(0.4,()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0,1);
                })
                .lineToLinearHeading(Constants.midParkRR)
                .lineToLinearHeading(Constants.closeParkOffsetRR)
                .lineToLinearHeading(Constants.closeParkRR)
                .build();

        while (!isStarted()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(Constants.startRR);

            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionAuton);
            sleep(800);

            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft);
                drive.followTrajectorySequence(yellowLeft);
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                drive.followTrajectorySequence(parkLeft);
            } else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                drive.followTrajectorySequence(parkCenter);
            } else {
                drive.followTrajectorySequence(purpleRight);
                dropdown.setPosition(Constants.dropdownPositionUp);
                sleep(300);
                drive.followTrajectorySequence(yellowRight);
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                drive.followTrajectorySequence(parkRight);
            }
//            sleep(2000);

            break;

        }
    }


}

