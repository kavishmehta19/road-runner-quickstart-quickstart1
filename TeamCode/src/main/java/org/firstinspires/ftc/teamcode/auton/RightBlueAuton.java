package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Right Blue Auton")

public class RightBlueAuton extends LinearOpMode {

    SampleMecanumDrive drive;//drivetrain

    DcMotor liftL;//lift motors
    DcMotor liftR;

    Servo blocker;

    Servo tiltL;
    Servo tiltR;
    Servo airplane;
    Servo dropdown;

    OpenCvCamera phoneCam;//camera
    WebcamName webcamName;
    OpenCVDebug.CenterStagePipeline pipeline;//opencv pipeline

    CRServo roller;

    TrajectoryVelocityConstraint v;
    TrajectoryAccelerationConstraint a;

    Pose2d startPose = Constants.startPoseBR;//positions
    Pose2d purplepixelcenterL = Constants.purplepixelcenterBR;

    Pose2d purplepixelleftL = Constants.purplepixelleftBR;
    Pose2d purplepixelcenterLoffset = Constants.purplepixelBRoffset;
    Pose2d yellowpixelcenterL = Constants.yellowpixelcenterBR;
    Pose2d parkL = Constants.parkL;

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

        blocker.setPosition(Constants.blockerClosedPosition);
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

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(startPose)//trajectory sequence
                .lineToLinearHeading(purplepixelcenterL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);
                })
                .lineToLinearHeading(Constants.purplepixelBRoffset)
                .build();

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.purplepixelleftBR)
                .lineToLinearHeading(Constants.purplepixelleftBRoffset)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);

                })
                .lineToLinearHeading(Constants.purplepixelBRoffset)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.purplepixelrightBR)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp);

                })
                .lineToLinearHeading(Constants.purplepixelBRoffset)
                .build();


        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(purplepixelcenterLoffset)
                .addTemporalMarker(2.3, ()->{
                    blocker.setPosition(Constants.blockerClosedPosition);
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(2.6, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.whitepixelBlue)
                .lineToLinearHeading(yellowpixelcenterL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purplepixelBRoffset)
                .addTemporalMarker(3.4, ()->{
                    blocker.setPosition(Constants.blockerClosedPosition);
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(3.8, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.whitepixelBlue)
                .lineToLinearHeading(Constants.yellowpixelleftBR)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purplepixelrightBL)
                .addTemporalMarker(3.4, ()->{
                    Constants.setLift(Constants.liftTargetLow, 1);
                })
                .addTemporalMarker(3.8, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.whitepixelBlue)
                .lineToLinearHeading(Constants.yellowpixelrightBR)
                .build();



        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(yellowpixelcenterL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetMid, 1);
                })

                .addTemporalMarker(0.4, ()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(parkL)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.yellowpixelleftBR)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetMid, 1);
                })
                .addTemporalMarker(0.4, ()->{
                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                    Constants.setLift(0, 1);
                })
                .lineToLinearHeading(parkL)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(Constants.yellowpixelrightBR)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetMid, 1);
                })
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
                drive.followTrajectorySequence(parkLeft);

            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                sleep(200);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(200);
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
                    .lineToLinearHeading(Constants.whitepixeloffsetBlueRight)
                    .addTemporalMarker(0.1,()->{
                        dropdown.setPosition(Constants.dropdownautonpositionstart + (currentCycleCounter * 0.03));
                        Constants.setIntake(1);
                        tiltL.setPosition(Constants.tiltIntakePositionL);
                        tiltR.setPosition(Constants.tiltIntakePositionR);
                        Constants.setLift(0, 1);
                    })
                    .build();
            TrajectorySequence drop = drive.trajectorySequenceBuilder(Constants.whitepixeloffsetBlueRight)
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
            drive.followTrajectorySequence(drop);
            Constants.setIntake(0);
            blocker.setPosition(Constants.blockerOpenPosition);
            sleep(300);
            drive.followTrajectorySequence(reset);
            cycleCounter++;
        }
    }




}

