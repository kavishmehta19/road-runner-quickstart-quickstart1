package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Color Sensor Debug")
public class ColorSensorDebug extends LinearOpMode {

        SampleMecanumDrive drive;

        ColorSensor color;

        DistanceSensor distance;

        @Override
        public void runOpMode() throws InterruptedException {
            drive = new SampleMecanumDrive(hardwareMap);

            color = hardwareMap.colorSensor.get("color");
//            distance = hardwareMap.get(DistanceSensor.class, "distance");


            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Red",color.red());
                telemetry.addData("Green",color.green());
                telemetry.addData("Blue",color.blue());
//                telemetry.addData("Distance",distance.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }


    }
