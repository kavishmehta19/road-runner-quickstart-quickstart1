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

@Autonomous(name="Right Blue Auton 2+2")

public class RightBlueAuton2 extends LinearOpMode {

    SampleMecanumDrive drive;//drivetrain

    //DECLARES VARIABLES
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

    Pose2d startPose = Constants.startPoseBR;//positions
    Pose2d purplepixelcenterL = Constants.purplepixelcenterBR; //Retrieves position from the purplepixelcenterBR in the constants class

    Pose2d purplepixelleftL = Constants.purplepixelleftBR;
    Pose2d purplepixelcenterLoffset = Constants.purplepixelcenterBRoffset;
    Pose2d yellowpixelcenterL = Constants.yellowpixelcenterBR;
    Pose2d parkL = Constants.parkL; //Retrieves parking from the constants class

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Constants.initHardware(hardwareMap); //Initializes Hardware Map

        //RETRIEVES VARIABLES FROM THE CONSTANTS CLASS
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

        pipeline = new OpenCVDebug.CenterStagePipeline(); //Sets pipeline for the camera to the CenterStage Pipeline
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //Starts streaming with a specified image size and rotation
            }

            @Override
            public void onError(int errorCode) { }
        });

        drive = new SampleMecanumDrive(hardwareMap); //Controls the robot's mecanum drive using the hardware map

        //PURPLE CENTER, LEFT, AND RIGHT TRAJECTORY SEQUENCES
        TrajectorySequence purpleCenter = drive.trajectorySequenceBuilder(startPose)//trajectory sequence
                .lineToLinearHeading(purplepixelcenterL) //Goes to purplepixelcenterL in a straight line trajectory
                .addDisplacementMarker(()->{ //Adds displacement marker - executes code when trajectory reaches a certain point
                    dropdown.setPosition(Constants.dropdownautonpositionstart); //Sets dropdown position to the constants class position
                    sleep(500); //sleeps for 0.5 seconds
                })
                .lineToLinearHeading(purplepixelcenterLoffset)
                .build(); //Builds trajectory sequence

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(purplepixelleftL)
                .addDisplacementMarker(()->{
                    dropdown.setPosition(Constants.dropdownPositionUp); //Dropdown position is up
                    sleep(300);
                })
                .lineToLinearHeading(Constants.purplepixelleftBLoffset)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(Constants.purplepixelrightBLoffset)
                .lineToLinearHeading(Constants.purplepixelrightBL)
                .build();

        //YELLOW CENTER, LEFT, AND RIGHT TRAJECTORY SEQUENCES
        TrajectorySequence yellowCenter = drive.trajectorySequenceBuilder(purplepixelcenterLoffset)
                .addTemporalMarker(2.3, ()->{ //executes below lines of code at 2.3 seconds
                    blocker.setPosition(Constants.blockerClosedPosition);
                    Constants.setLift(Constants.liftTargetAuton, 1);
                })
                .addTemporalMarker(2.6, ()->{ //executes below lines of code at 2.6 seconds
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.whitepixelBlue)
                .lineToLinearHeading(yellowpixelcenterL)
                .build();

        TrajectorySequence yellowLeft = drive.trajectorySequenceBuilder(Constants.purplepixelleftBLoffset)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetLow, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.yellowpixelleftBL)
                .build();

        TrajectorySequence yellowRight = drive.trajectorySequenceBuilder(Constants.purplepixelrightBL)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetLow, 1);
                })
                .addTemporalMarker(0.3, ()->{
                    tiltL.setPosition(Constants.tiltDropPositionL);
                    tiltR.setPosition(Constants.tiltDropPositionR);
                })
                .lineToLinearHeading(Constants.yellowpixelrightBL)
                .build();


        //PARK CENTER, LEFT, AND RIGHT TRAJECTORY SEQUENCES
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

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(Constants.yellowpixelleftBL)
                .addTemporalMarker(0.4, ()->{
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




        waitForStart(); // Waits for start button to be pressed on driver station

        if (isStopRequested()) return; //exits the method if stop button was pressed

        while (opModeIsActive() && !isStopRequested()) { //Runs while opmode is active, and stop has not been requested

            drive.setPoseEstimate(startPose); //sets pose estimate to start pose

            OpenCVDebug.CenterStagePipeline.Position position = pipeline.position; //Gets current position value from pipeline associated with camera
            telemetry.addData("Analysis", pipeline.getAnalysis());//Adds data to telemetry log
            telemetry.addData("Position", pipeline.position); //Adds data to telemetry log
            telemetry.update();

            dropdown.setPosition(Constants.dropdownPositionDown); //Sets dropdown position to down
            sleep(500);

            //EXECUTES CODE FOR RESPECTIVE POSITIONS
            if (position == OpenCVDebug.CenterStagePipeline.Position.LEFT) {
                drive.followTrajectorySequence(purpleLeft); //follows trajectory for purple left
                drive.followTrajectorySequence(yellowLeft); //follows trajectory for yellow left
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition); //opens blocker at the end
                sleep(500);
                drive.followTrajectorySequence(parkLeft); //parks
            }
            else if (position == OpenCVDebug.CenterStagePipeline.Position.CENTER) {
                drive.followTrajectorySequence(purpleCenter);
                drive.followTrajectorySequence(yellowCenter);
                sleep(500);
                blocker.setPosition(Constants.blockerOpenPosition);
                sleep(500);
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



}

