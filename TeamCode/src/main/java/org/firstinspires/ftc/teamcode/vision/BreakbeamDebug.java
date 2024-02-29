package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class BreakbeamDebug extends LinearOpMode {

    DigitalChannel beam1;
 
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");

        beam1.setMode(DigitalChannel.Mode.INPUT);


        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Status", "Running");

            telemetry.addData("beam1", beam1.getState());

            telemetry.update();

        }

    }

}