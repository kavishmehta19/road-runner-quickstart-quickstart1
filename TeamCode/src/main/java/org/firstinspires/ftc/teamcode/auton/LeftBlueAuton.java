package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Left Blue Auton")

public class LeftBlueAuton extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;

    DcMotor intakeL;

    DcMotor intakeR;

    Servo blocker;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;

    TrajectoryVelocityConstraint v;
    TrajectoryAccelerationConstraint a;

    CRServo roller;

    int numcycles = 3;

    Pose2d startPose = Constants.startPoseBL;
    Pose2d purplepixelcenterL = Constants.purplepixelcenterBL;

    Pose2d purplepixelleftL = Constants.purplepixelleftBL;
    Pose2d purplepixelcenterLoffset = Constants.purplepixelcenterBLoffset;
    Pose2d yellowpixelcenterL = Constants.yellowpixelcenterBL;
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
        intakeL = Constants.intakeL;
        intakeR = Constants.intakeR;

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

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(purplepixelcenterL, purplepixelcenterL.getHeading())
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .splineToSplineHeading(purplepixelcenterLoffset, purplepixelcenterL.getHeading())
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(purplepixelleftL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purplepixelleftBLoffset)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.purplepixelrightBLoffset)
                .lineToLinearHeading(Constants.purplepixelrightBL)
                .build();

        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(purplepixelcenterLoffset)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(yellowpixelcenterL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purplepixelleftBLoffset)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.yellowpixelleftBL)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purplepixelrightBL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.yellowpixelrightBL)
                .build();



        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(yellowpixelcenterL)
                .addTemporalMarker(0.4, ()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(parkL)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.yellowpixelleftBL)
                .addTemporalMarker(0.2, ()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(parkL)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.yellowpixelrightBL)
                .addTemporalMarker(0.4, ()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(parkL)
                .build();




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPoseEstimate(startPose);

            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionDown);
            sleep(500);

            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft);
                drive.followTrajectorySequence(yellowLeft);
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                Constants.setLift(Constants.liftTargetAuton - 400, 1);
                drive.followTrajectorySequence(parkLeft);
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                Constants.setLift(Constants.liftTargetAuton - 400, 1);
                drive.followTrajectorySequence(parkCenter);
            }
            else {
                drive.followTrajectorySequence(purpleRight);
                dropdown.setPosition(Constants.dropdownPositionUp);
                sleep(300);
                drive.followTrajectorySequence(yellowRight);
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
                Constants.setLift(Constants.liftTargetAuton - 400, 1);
                drive.followTrajectorySequence(parkRight);
            }
            sleep(2000);

            break;

        }
    }

    public void runCycles(SampleMecanumDrive drive) {
        int cycleCounter = 0;
        while (cycleCounter < numcycles && opModeIsActive()) {
            int currentCycleCounter = cycleCounter;
            TrajectorySequence pickup = drive.trajectorySequenceBuilder(parkL)
                    .setVelConstraint(v)
                    .setAccelConstraint(a)
                    .lineToLinearHeading(Constants.whitepixelBlue)
                    .lineToLinearHeading(Constants.whitepixeloffsetBlueLeft)
                    .addTemporalMarker(0.1,()->{
                        dropdown.setPosition(Constants.dropdownautonpositionstart + (currentCycleCounter * 0.03));
                        Constants.setIntake(0.6);
                        tiltL.setPosition(Constants.tiltIntakePositionL);
                        tiltR.setPosition(Constants.tiltIntakePositionR);
                        Constants.setLift(0, 1);
                    })
                    .build();
            TrajectorySequence drop = drive.trajectorySequenceBuilder(Constants.whitepixeloffsetBlueLeft)
                    .addTemporalMarker(2.5, ()->{
                        Constants.setLift(Constants.liftTargetMid, 1);
                        blocker.setPosition(Constants.blockerClosedPosition);
                    })
                    .addTemporalMarker(3.5, ()->{
                        tiltL.setPosition(Constants.tiltDropPositionL);
                        tiltR.setPosition(Constants.tiltDropPositionR);
                    })
                    .lineToLinearHeading(Constants.whitepixelBlue)
                    .addDisplacementMarker(()->{
                        Constants.setIntake(0);
                    })
                    .lineToLinearHeading(yellowpixelcenterL)
                    .build();

            TrajectorySequence reset = drive.trajectorySequenceBuilder(yellowpixelcenterL)
                    .resetConstraints()
                    .lineToLinearHeading(parkL)
                    .addTemporalMarker(0.2, ()->{
                        Constants.setLift(Constants.liftTargetMid - 300,1);
                    })
                    .addTemporalMarker(0.6, ()->{
                        tiltL.setPosition(Constants.tiltIntakePositionL);
                        tiltR.setPosition(Constants.tiltIntakePositionR);

                    })
                    .addTemporalMarker(0.8, ()->{
                        Constants.setLift(0, 1);
                    })
                    .build();

            drive.followTrajectorySequence(pickup);
            cycleCounter++;
            sleep(500);
            dropdown.setPosition(Constants.dropdownautonpositionstart + cycleCounter * 0.03);
            sleep(1000);
            drive.followTrajectorySequence(drop);
            Constants.setIntake(0);
            blocker.setPosition(Constants.blockerOpenPosition);
            sleep(300);
            drive.followTrajectorySequence(reset);
            cycleCounter++;
        }
    }


}

