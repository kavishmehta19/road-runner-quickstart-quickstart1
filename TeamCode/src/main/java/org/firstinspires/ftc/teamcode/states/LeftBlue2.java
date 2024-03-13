package org.firstinspires.ftc.teamcode.states;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Left Blue Auton 2+2")

public class LeftBlue2 extends LinearOpMode {

    //DECLARING VARIABLES
    SampleMecanumDrive drive; //Driving Code

    DcMotor liftL;
    DcMotor liftR;

    DcMotor intakeL;

    DcMotor intakeR;

    Servo blocker;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    DigitalChannel beamBreak;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;

    OpenCVDebug.CenterStagePipeline.Position position;

    TrajectoryVelocityConstraint v;
    TrajectoryAccelerationConstraint a;

    CRServo roller;

    int numcycles = 2;

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
        intakeL = Constants.intakeL;
        intakeR = Constants.intakeR;

        beamBreak = Constants.beamBreak;

        blocker.setPosition(Constants.blockerClosedPosition);
        dropdown.setPosition(Constants.dropdownPositionUp);

        v = new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                return 50;
            }
        };

        a = new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                return 50;
            }
        };

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

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(Constants.startBL)
                .lineToLinearHeading(Constants.purpleCenterOffsetBL)
                .lineToLinearHeading(Constants.purpleCenterBL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleCenterOffsetBL)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(Constants.startBL)
                .lineToLinearHeading(Constants.purpleLeftOffsetBL)
                .lineToLinearHeading(Constants.purpleLeftBL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleLeftOffsetBL)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(Constants.startBL)
                .lineToLinearHeading(Constants.purpleRightOffsetBL)
                .lineToLinearHeading(Constants.purpleRightBL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purpleRightOffsetBL)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(Constants.purpleCenterOffsetBL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.4, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .lineToLinearHeading(Constants.backdropCenterBL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purpleLeftOffsetBL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .lineToLinearHeading(Constants.backdropLeftBL)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purpleRightOffsetBL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .lineToLinearHeading(Constants.backdropRightBL)
                .build();



        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(Constants.backdropCenterBL)
                .addTemporalMarker(0.01, ()->{
                    Constants.setLift(Constants.liftTargetMid,1);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.backdropLeftBL)
                .addTemporalMarker(0.01, ()->{
                    Constants.setLift(Constants.liftTargetMid,1);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.backdropRightBL)
                .addTemporalMarker(0.01, ()->{
                    Constants.setLift(Constants.liftTargetMid,1);
                })
                .lineToLinearHeading(Constants.midParkBL)
                .build();

        while (!isStarted()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(Constants.startBL);

            position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionAuton);
            sleep(600);

            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft);
                drive.followTrajectorySequence(yellowLeft);
                sleep(200);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(200);
                drive.followTrajectorySequence(parkLeft);
                runCycles(drive);
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                sleep(200);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(200);
                drive.followTrajectorySequence(parkCenter);
                runCycles(drive);
            }
            else {
                drive.followTrajectorySequence(purpleRight);
                drive.followTrajectorySequence(yellowRight);
                sleep(200);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(200);
                drive.followTrajectorySequence(parkRight);
                runCycles(drive);
            }
            sleep(2000);

            break;

        }
    }

    public void runCycles(SampleMecanumDrive drive) {
        int cycleCounter = 0;
        while (cycleCounter < numcycles && opModeIsActive()) {
            int currentCycleCounter = cycleCounter;
            TrajectorySequence pickup = drive.trajectorySequenceBuilder(Constants.midParkBL)
                    .lineToLinearHeading(Constants.crossOffsetBL)
                    .lineToLinearHeading(Constants.whitePickupBL)
                    .addTemporalMarker(0.1,()->{
                        tiltL.setPosition(Constants.tiltIntakePositionL);
                        tiltR.setPosition(Constants.tiltIntakePositionR);
                        Constants.setLift(0, 1);
                    })
                    .addTemporalMarker(1.5,()->{
                        dropdown.setPosition(Constants.dropdownIntakeStart + (4 * 0.025));
                        Constants.setIntake(-1);
                        blocker.setPosition(Constants.blockerWidePosition);
                    })
                    .addTemporalMarker(3.0,()->{
                        dropdown.setPosition(Constants.dropdownIntakeStart + (currentCycleCounter * Constants.dropdownIncrement));
                        Constants.setIntake(0.9);
                    })
                    .build();
            TrajectorySequence intakeOffset = drive.trajectorySequenceBuilder(Constants.whitePickupBL)
                    .addTemporalMarker(0.4,()->{
                        dropdown.setPosition(Constants.dropdownPositionDown);
                    })
                    .lineToLinearHeading(Constants.whitePickupOffsetBL)
                    .build();

            TrajectorySequence drop = drive.trajectorySequenceBuilder(Constants.whitePickupOffsetBL)
                    .addTemporalMarker(0.4,()->{
                        dropdown.setPosition(Constants.dropdownPositionDown);
                    })
                    .addTemporalMarker(2.5, ()->{
                        Constants.setLift(Constants.liftTargetAuton2, 1);
                        blocker.setPosition(Constants.blockerClosedPosition);
                    })
                    .addTemporalMarker(3.5, ()->{
                        tiltL.setPosition(Constants.tiltDropPositionL);
                        tiltR.setPosition(Constants.tiltDropPositionR);
                    })
                    .lineToLinearHeading(Constants.crossOffsetBL)
                    .addDisplacementMarker(()->{
                        Constants.setIntake(0);
                        dropdown.setPosition(Constants.dropdownPositionUp);
                    })
//                    .lineToLinearHeading(Constants.midParkBL)
                    .lineToLinearHeading(position == OpenCVDebug.CenterStagePipeline.Position.RIGHT ? Constants.backdropLeftBL : Constants.backdropRightBL)
                    .build();

            TrajectorySequence reset = drive.trajectorySequenceBuilder(position == OpenCVDebug.CenterStagePipeline.Position.RIGHT ? Constants.backdropLeftBL : Constants.backdropRightBL)
                    .lineToLinearHeading(Constants.midParkBL)
                    .addTemporalMarker(0.01, ()->{
                        Constants.setLift(Constants.liftTargetHigh,1);
                    })
                    .addTemporalMarker(0.7, ()->{
                        tiltL.setPosition(Constants.tiltIntakePositionL);
                        tiltR.setPosition(Constants.tiltIntakePositionR);

                    })
                    .addTemporalMarker(1, ()->{
                        Constants.setLift(0, 1);
                    })
                    .build();

            drive.followTrajectorySequence(pickup);
            cycleCounter++;
            Constants.setIntake(1);
            if (sleepCheck(250) > 0) {
                Constants.setIntake(-1);
                blocker.setPosition(Constants.blockerClosedPosition);
            }
            else {
                dropdown.setPosition(Constants.dropdownIntakeStart + cycleCounter * Constants.dropdownIncrement);
                if (sleepCheck(500) > 0){
                    Constants.setIntake(-1);
                    blocker.setPosition(Constants.blockerClosedPosition);
                }
            }
            drive.followTrajectorySequence(intakeOffset);
            if (sleepCheck(300) > 0) {
                Constants.setIntake(-1);
                blocker.setPosition(Constants.blockerClosedPosition);
            }
            drive.followTrajectorySequence(drop);
            Constants.setIntake(0);
            sleep(300);
            blocker.setPosition(Constants.blockerOpenPosition);
            sleep(300);
            drive.followTrajectorySequence(reset);
            cycleCounter++;
        }
    }

    public int sleepCheck(int millis) {
        boolean timerStarted = false;
        long timer = System.currentTimeMillis();

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < millis) {
            if (!beamBreak.getState() && !timerStarted) {
                timerStarted = true;
                timer = System.currentTimeMillis();
            } else if (!beamBreak.getState()) {
                if (System.currentTimeMillis() - timer > 700) {
                    return 1;
                }
            } else {
                timerStarted = false;
            }
        }
        return 0;
    }


}

