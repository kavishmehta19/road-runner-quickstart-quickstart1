//package org.firstinspires.ftc.teamcode.unused;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
//import com.acmerobotics.roadrunner.profile.VelocityConstraint;
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
//import org.checkerframework.checker.units.qual.A;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Disabled
//@Autonomous(name="Left Blue Auton No Beam 2+2")
//
//public class LeftBlueAuton2 extends LinearOpMode {
//
//    //DECLARING VARIABLES
//    SampleMecanumDrive drive; //Driving Code
//
//    DcMotor liftL;
//    DcMotor liftR;
//
//    DcMotor intakeL;
//
//    DcMotor intakeR;
//
//    Servo blocker;
//
//    Servo tiltL;
//    Servo tiltR;
//    Servo airplane;
//    Servo dropdown;
//
//    OpenCvCamera phoneCam;
//    WebcamName webcamName;
//    OpenCVDebug.CenterStagePipeline pipeline;
//
//    TrajectoryVelocityConstraint v;
//    TrajectoryAccelerationConstraint a;
//
//    CRServo roller;
//
//    int numcycles = 2;
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
//        intakeL = Constants.intakeL;
//        intakeR = Constants.intakeR;
//
//        blocker.setPosition(Constants.blockerClosedPosition);
//        dropdown.setPosition(Constants.dropdownPositionUp);
//
//        v = new TrajectoryVelocityConstraint() {
//            @Override
//            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
//                return 50;
//            }
//        };
//
//        a = new TrajectoryAccelerationConstraint() {
//            @Override
//            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
//                return 50;
//            }
//        };
//
//        webcamName = hardwareMap.get(WebcamName.class, "webcam");
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
//        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startBL)
//                .lineToLinearHeading(Constants.purpleCenterBL)
//                .addDisplacementMarker(()->{
//                    dropdown.setPosition(Constants.dropdownPositionUp);
//                    sleep(300);
//                })
//                .lineToLinearHeading(Constants.purpleCenterOffsetBL)
//                .build();
//
//        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startBL)
//                .lineToLinearHeading(Constants.purpleLeftBL)
//                .addDisplacementMarker(()->{
//                    dropdown.setPosition(Constants.dropdownPositionUp);
//                    sleep(300);
//                })
//                .lineToLinearHeading(Constants.purpleLeftOffsetBL)
//                .build();
//
//        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startBL)
//                .lineToLinearHeading(Constants.purpleRightOffsetBL)
//                .lineToLinearHeading(Constants.purpleRightBL)
//                .build();
//
//        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleCenterOffsetBL)
//                .addTemporalMarker(0.1, ()->{
//                    Constants.setLift(Constants.liftTargetAuton, 1);
//                })
//                .addTemporalMarker(0.4, ()->{
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.backdropCenterBL)
//                .build();
//
//        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleLeftOffsetBL)
//                .addTemporalMarker(0.1, ()->{
//                    Constants.setLift(Constants.liftTargetAuton, 1);
//                })
//                .addTemporalMarker(0.3, ()->{
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.backdropLeftBL)
//                .build();
//
//        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleRightBL)
//                .addTemporalMarker(0.1, ()->{
//                    Constants.setLift(Constants.liftTargetAuton, 1);
//                })
//                .addTemporalMarker(0.3, ()->{
//                    tiltL.setPosition(Constants.tiltDropPositionL);
//                    tiltR.setPosition(Constants.tiltDropPositionR);
//                })
//                .lineToLinearHeading(Constants.backdropRightBL)
//                .build();
//
//
//
//        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(Constants.backdropCenterBL)
//                .addTemporalMarker(0.2, ()->{
//                    Constants.setLift(Constants.liftTargetMid,1);
//                })
//                .lineToLinearHeading(Constants.midParkBL)
//                .build();
//
//        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.backdropLeftBL)
//                .addTemporalMarker(0.01, ()->{
//                    Constants.setLift(Constants.liftTargetMid - 200,1);
//                })
//                .lineToLinearHeading(Constants.midParkBL)
//                .build();
//
//        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.backdropRightBL)
//                .addTemporalMarker(0.2, ()->{
//                    Constants.setLift(Constants.liftTargetMid,1);
//                })
//                .lineToLinearHeading(Constants.midParkBL)
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
//            drive.setPoseEstimate(Constants.startBL);
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
//                sleep(500);
//                blocker.setPosition(Constants.blockerOpenPosition);
//                sleep(500);
//                drive.followTrajectorySequence(parkLeft);
//                runCycles(drive);
//            }
//            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
//                drive.followTrajectorySequence(purpleCenter);
//                drive.followTrajectorySequence(yellowCenter);
//                blocker.setPosition(Constants.blockerOpenPosition);
//                drive.followTrajectorySequence(parkCenter);
//                runCycles(drive);
//            }
//            else {
//                drive.followTrajectorySequence(purpleRight);
//                dropdown.setPosition(Constants.dropdownPositionUp);
//                sleep(300);
//                drive.followTrajectorySequence(yellowRight);
//                sleep(500);
//                blocker.setPosition(Constants.blockerOpenPosition);
//                sleep(500);
//                drive.followTrajectorySequence(parkRight);
//                runCycles(drive);
//            }
//            sleep(2000);
//
//            break;
//
//        }
//    }
//
//    public void runCycles(SampleMecanumDrive drive) {
//        int cycleCounter = 0;
//        while (cycleCounter < numcycles && opModeIsActive()) {
//            int currentCycleCounter = cycleCounter;
//            TrajectorySequence pickup = drive.trajectorySequenceBuilder(Constants.midParkBL)
//                    .setVelConstraint(v)
//                    .setAccelConstraint(a)
//                    .lineToLinearHeading(Constants.crossOffsetBL)
//                    .lineToLinearHeading(Constants.whitePickupBL)
//                    .addTemporalMarker(0.1,()->{
//                        tiltL.setPosition(Constants.tiltIntakePositionL);
//                        tiltR.setPosition(Constants.tiltIntakePositionR);
//                        Constants.setLift(0, 1);
//                    })
//                    .addTemporalMarker(1.5,()->{
//                        dropdown.setPosition(Constants.dropdownIntakeStart + (4 * 0.025));
//                        Constants.setIntake(-1);
//                        blocker.setPosition(Constants.blockerWidePosition);
//                    })
//                    .addTemporalMarker(3.0,()->{
//                        dropdown.setPosition(Constants.dropdownIntakeStart + (currentCycleCounter * Constants.dropdownIncrement));
//                        Constants.setIntake(0.9);
//                    })
//                    .build();
//            TrajectorySequence drop = drive.trajectorySequenceBuilder(Constants.whitePickupBL)
//                    .addTemporalMarker(0.4,()->{
//                        dropdown.setPosition(Constants.dropdownPositionDown);
//                    })
//                    .addTemporalMarker(2.5, ()->{
//                        Constants.setLift(Constants.liftTargetMid - 200, 1);
//                        blocker.setPosition(Constants.blockerClosedPosition);
//                    })
//                    .addTemporalMarker(3.5, ()->{
//                        tiltL.setPosition(Constants.tiltDropPositionL);
//                        tiltR.setPosition(Constants.tiltDropPositionR);
//                    })
//                    .lineToLinearHeading(Constants.crossOffsetBL)
//                    .addDisplacementMarker(()->{
//                        Constants.setIntake(0);
//                    })
//                    .lineToLinearHeading(Constants.backdropLeftBL)
//                    .build();
//
//            TrajectorySequence reset = drive.trajectorySequenceBuilder(Constants.backdropLeftBL)
//                    .resetConstraints()
//                    .lineToLinearHeading(Constants.midParkBL)
//                    .addTemporalMarker(0.2, ()->{
//                        Constants.setLift(Constants.liftTargetMid - 500,1);
//                    })
//                    .addTemporalMarker(0.7, ()->{
//                        tiltL.setPosition(Constants.tiltIntakePositionL);
//                        tiltR.setPosition(Constants.tiltIntakePositionR);
//
//                    })
//                    .addTemporalMarker(1, ()->{
//                        Constants.setLift(0, 1);
//                    })
//                    .build();
//
//            drive.followTrajectorySequence(pickup);
//            cycleCounter++;
//            Constants.setIntake(1);
//            sleep(500);
//            dropdown.setPosition(Constants.dropdownIntakeStart + cycleCounter * Constants.dropdownIncrement);
//            sleep(1000);
//            drive.followTrajectorySequence(drop);
//            Constants.setIntake(0);
//            blocker.setPosition(Constants.blockerOpenPosition);
//            sleep(300);
//            drive.followTrajectorySequence(reset);
//            cycleCounter++;
//        }
//    }
//
//
//}
//
