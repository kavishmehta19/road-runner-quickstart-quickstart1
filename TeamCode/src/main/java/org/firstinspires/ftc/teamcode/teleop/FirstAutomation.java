package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="First Automation")
public class FirstAutomation extends LinearOpMode {

    SampleMecanumDrive drive;
    DcMotor liftL = null;
    DcMotor liftR = null;

    DcMotor intakeR = null;

    DcMotor intakeL = null;


    Servo blocker = null;

    Servo tiltL = null;
    Servo tiltR = null;

    CRServo roller = null;

    Servo airplane = null;
    Servo dropdown = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        g1 = gamepad1;
        g2 = gamepad2;

        g1.type = Gamepad.Type.SONY_PS4;
        g2.type = Gamepad.Type.SONY_PS4;

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;

        intakeR = Constants.intakeR;
        intakeL = Constants.intakeL;

        blocker = Constants.blocker;
        roller = Constants.roller;
        tiltL = Constants.tiltL;
        tiltR = Constants.tiltR;
        airplane = Constants.airplane;
        dropdown = Constants.dropdown;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double tiltIntakePositionL = 0.8;
        double tiltIntakePositionR = 0.18;

        double tiltDropPositionL = 0.2;
        double tiltDropPositionR = 0.75;

        double blockerOpenPosition = 0.5;
        double blockerClosedPosition = 1;

        double airplaneOpenPosition = 1;
        double airplaneClosedPosition = 0;

        double dropdownPosition = Constants.dropdownPositionUp;
        dropdown.setPosition(Constants.dropdownPositionUp);

        double tiltRPosition = tiltIntakePositionR;
        double tiltLPosition = tiltIntakePositionL;

        long timer = System.currentTimeMillis();
        boolean timerRunning = false;

        int liftTarget = 0;
        boolean liftHittingTarget = false;

        boolean depositing = false;

        boolean tiltDropping = false;
        tiltL.setPosition(tiltIntakePositionL);
        tiltR.setPosition(tiltIntakePositionR);
        airplane.setPosition(airplaneClosedPosition);

        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;

        boolean blockerOpen = true;
        blocker.setPosition(blockerOpenPosition);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double y = -g1.left_stick_y;
            double x = -g1.left_stick_x;
            double turn = -g1.right_stick_x;

            double yMultiplier = 1;
            double xMultiplier = 1;
            double turnMultiplier = 0.7;

            if (Math.abs(x) < 0.4 && Math.abs(y) > 0.5)
                x = 0;


            if (y < 0) yMultiplier *= -1;
            if (x < 0) xMultiplier *= -1;
            if (turn < 0) turnMultiplier *= -1;

            y = Math.pow(y, 2) * yMultiplier;
            x = Math.pow(x, 2) * xMultiplier;
            turn = Math.pow(turn, 2) * turnMultiplier;

            drive.setWeightedDrivePower(new Pose2d(y, x, turn));

            intakeR.setPower(g2.right_trigger - g2.left_trigger);
            intakeL.setPower(-(g2.right_trigger - g2.left_trigger));

            roller.setPower(-g2.right_trigger + g2.left_trigger);

            boolean rightBumper = g2.right_bumper;
            if (rightBumper && !lastRightBumper) {
                if (blockerOpen) {
                    blocker.setPosition(blockerClosedPosition);
                } else {
                    blocker.setPosition(blockerOpenPosition);
                }
                blockerOpen = !blockerOpen;

            }
            lastRightBumper = rightBumper;

            boolean leftBumper = g2.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                if (tiltDropping) {
                    tiltL.setPosition(tiltIntakePositionL);
                    tiltR.setPosition(tiltIntakePositionR);
                } else {
                    tiltL.setPosition(tiltDropPositionL);
                    tiltR.setPosition(tiltDropPositionR);
                }
                tiltDropping = !tiltDropping;
            }

            if (g2.square) {
                liftTarget = Constants.liftTargetLow;
                liftHittingTarget = true;
                depositing = true;
            }
            else if (g2.circle) {
                liftTarget = Constants.liftTargetMid;
                liftHittingTarget = true;
                depositing = true;
            }
            else if (g2.triangle) {
                liftTarget = Constants.liftTargetHigh;
                liftHittingTarget = true;
                depositing = true;
            }
            else if (g2.cross) {
                liftTarget = 0;
                liftHittingTarget = true;
                depositing = false;
            }

            if ((liftHittingTarget && Math.abs(liftTarget - liftL.getCurrentPosition()) > 20) && !g2.right_stick_button) {
                Constants.setLift(liftTarget, 1);
                if (depositing){
                    blocker.setPosition(blockerClosedPosition);
                    blockerOpen = false;

                    if (!timerRunning) {
                        timer = System.currentTimeMillis();
                        timerRunning = true;
                    }
                }
                else {
                    blocker.setPosition(blockerOpenPosition);
                    blockerOpen = true;

                    tiltL.setPosition(Constants.tiltIntakePositionL);
                    tiltR.setPosition(Constants.tiltIntakePositionR);
                }
            } else {
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                liftL.setPower(g2.left_stick_y);
                liftR.setPower(-g2.left_stick_y);

                liftHittingTarget = false;
            }

            if (timerRunning && System.currentTimeMillis() - timer > 300){
                tiltL.setPosition(Constants.tiltDropPositionL);
                tiltR.setPosition(Constants.tiltDropPositionR);
                timerRunning = false;
            }

            if(g1.a){
                airplane.setPosition(airplaneOpenPosition);
            }
            else{
                airplane.setPosition(airplaneClosedPosition);
            }

            if (g2.dpad_left){
                dropdownPosition = Constants.dropdownPositionUp;
            }
            if (g2.dpad_right){
                dropdownPosition = Constants.dropdownPositionDown;
            }

            if (g2.dpad_up){
                dropdownPosition -= 0.001;
            }
            if (g2.dpad_down){
                dropdownPosition += 0.001;
            }

            dropdown.setPosition(dropdownPosition);

            lastLeftBumper = leftBumper;
            telemetry.addData("liftR", liftR.getCurrentPosition());
            telemetry.addData("liftL", liftL.getCurrentPosition());
            telemetry.addData("tiltR", tiltR.getPosition());
            telemetry.addData("tiltL", tiltL.getPosition());
            telemetry.addData("dropdown", dropdown.getPosition());
            telemetry.addData("airplane", airplane.getPosition());
            telemetry.update();

        }

    }

}
