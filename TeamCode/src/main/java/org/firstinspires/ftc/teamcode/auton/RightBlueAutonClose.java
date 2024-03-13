//package org.firstinspires.ftc.teamcode.auton;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.apache.commons.math3.analysis.function.Constant;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Autonomous(name="Right Blue Auton Close 2+0")
//
//public class RightBlueAutonClose extends LinearOpMode {
//
//    SampleMecanumDrive drive;//drivetrain
//
//    DcMotor liftL;//lift motors
//    DcMotor liftR;
//
//    Servo blocker;
//
//    Servo tiltL;
//    Servo tiltR;
//    Servo airplane;
//    Servo dropdown;
//
//    OpenCvCamera phoneCam;//camera
//    WebcamName webcamName;
//    OpenCVDebug.CenterStagePipeline pipeline;//opencv pipeline
//
//    CRServo roller;
//
//    int numCycles = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        Constants.initHardware(hardwareMap);
//
//        liftL = Constants.liftL;
//        liftR = Constants.liftR;
//
//        blocker = Constants.blocker;
//        roller = Constants.roller;
//        tiltL = Constants.tiltL;
//        tiltR = Constants.tiltR;
//        airplane = Constants.airplane;
//        dropdown = Constants.dropdown;
//
//        blocker.setPosition(Constants.blockerClosedPosition);
//        dropdown.setPosition(Constants.dropdownPositionUp);
//
//        webcamName = hardwareMap.get(WebcamName.class, "webcam");//opencv
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        pipeline = new OpenCVDebug.CenterStagePipeline();
//        phoneCam.setPipeline(pipeline);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) { }
//        });
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startBR)//trajectory sequence
//                .lineToLinearHeading(Constants.purpleCloseCenterBR)
//                .addDisplacementMarker(()->{
//                    dropdown.setPosition(Constants.dropdownPositionUp);
//                })
//                .lineToLinearHeading(Constants.purpleCloseOffsetBR)
//                .build();
//
//        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startBR)
//                .lineToLinearHeading(Constants.purpleLeftOffsetBR)
//                .lineToLinearHeading(Constants.purpleLeftBR)
//                .addDisplacementMarker(()->{
//                    dropdown.setPosition(Constants.dropdownPositionUp);
//                })
//                .lineToLinearHeading(Constants.purpleLeftOffsetBR)
//                .lineToLinearHeading(Constants.purpleCloseLeftOffsetBR)
//                .build();
//
//        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startBR)
//                .lineToLinearHeading(Constants.purpleCloseRightBR)
//                .addDisplacementMarker(()->{
//                    dropdown.setPosition(Constants.dropdownPositionUp);
//
//                })
//                .lineToLinearHeading(Constants.purpleCloseOffsetBR)
//                .build();
//
//
//        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
//                .addTemporalMarker(2.5, () -> {
//                    Constants.setLift(Constants.liftTargetLow, 1);
//                    blocker.setPosition(Constants.blockerClosedPosition);
//                })
//                .addTemporalMarker(3.5, () -> {
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.prepareDropCloseBR)
//                .addDisplacementMarker(() -> {
//                    Constants.setIntake(0);
//                })
//                .lineToLinearHeading(Constants.yellowCloseResetBR)
//                .lineToLinearHeading(Constants.yellowCloseCenterBR)
//                .addDisplacementMarker(()->{
//                    blocker.setPosition(Constants.blockerOpenPosition);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
//                    Constants.setLift(Constants.liftTargetMid, 1);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    tiltL.setPosition(Constants.tiltIntakePositionL);
//                    tiltR.setPosition(Constants.tiltIntakePositionR);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    Constants.setLift(0, 1);
//                })
//
//                .lineToLinearHeading(Constants.yellowResetBR)
//                .lineToLinearHeading(Constants.closeParkOffsetBR)
//                .lineToLinearHeading(Constants.closeParkBR)
//                .build();
//
//        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleCloseLeftOffsetBR)
//                .addTemporalMarker(2.5, () -> {
//                    Constants.setLift(Constants.liftTargetLow, 1);
//                    blocker.setPosition(Constants.blockerClosedPosition);
//                })
//                .addTemporalMarker(3.5, () -> {
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.prepareDropCloseLeftBR)
//                .addDisplacementMarker(() -> {
//                    Constants.setIntake(0);
//                })
//                .lineToLinearHeading(Constants.yellowCloseResetBR)
//                .lineToLinearHeading(Constants.yellowCloseLeftBR)
//                .addDisplacementMarker(()->{
//                    blocker.setPosition(Constants.blockerOpenPosition);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
//                    Constants.setLift(Constants.liftTargetMid, 1);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    tiltL.setPosition(Constants.tiltIntakePositionL);
//                    tiltR.setPosition(Constants.tiltIntakePositionR);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    Constants.setLift(0, 1);
//                })
//                .lineToLinearHeading(Constants.yellowResetBR)
//                .lineToLinearHeading(Constants.closeParkOffsetLeftBR)
//                .lineToLinearHeading(Constants.closeParkLeftBR)
//                .build();
//
//        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleCloseOffsetBR)
//                .addTemporalMarker(2.5, () -> {
//                    Constants.setLift(Constants.liftTargetLow, 1);
//                    blocker.setPosition(Constants.blockerClosedPosition);
//                })
//                .addTemporalMarker(3.5, () -> {
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.prepareDropCloseBR)
//                .addDisplacementMarker(() -> {
//                    Constants.setIntake(0);
//                })
//                .lineToLinearHeading(Constants.yellowCloseResetBR)
//                .lineToLinearHeading(Constants.yellowCloseRightBR)
//                .addDisplacementMarker(()->{
//                    blocker.setPosition(Constants.blockerOpenPosition);
//                    sleep(500);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.01,()->{
//                    Constants.setLift(Constants.liftTargetMid, 1);
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    tiltL.setPosition(Constants.tiltIntakePositionL);
//                    tiltR.setPosition(Constants.tiltIntakePositionR);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    Constants.setLift(0, 1);
//                })
//                .lineToLinearHeading(Constants.yellowResetBR)
//                .lineToLinearHeading(Constants.closeParkOffsetBR)
//                .lineToLinearHeading(Constants.closeParkBR)
//                .build();
//
//        while (!isStarted()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            drive.setPoseEstimate(Constants.startBR);
//
//            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position;
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//
//            dropdown.setPosition(Constants.dropdownPositionAuton);
//            sleep(800);
//
//            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
//                drive.followTrajectorySequence(purpleLeft);
//                drive.followTrajectorySequence(yellowLeft);
//
//
//            }
//            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
//                drive.followTrajectorySequence(purpleCenter);
//                drive.followTrajectorySequence(yellowCenter);
//
//
//            }
//            else {
//                drive.followTrajectorySequence(purpleRight);
//                dropdown.setPosition(Constants.dropdownPositionUp);
//                sleep(300);
//                drive.followTrajectorySequence(yellowRight);
//            }
//            sleep(2000);
//
//            break;
//
//        }
//    }
//}
//
