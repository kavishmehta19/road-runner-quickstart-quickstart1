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

@TeleOp(name="Regionals")
public class SecondAutomation extends LinearOpMode {

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

        double airplaneOpenPosition = 1;
        double airplaneClosedPosition = 0;

        double tiltPositionL = Constants.tiltIntakePositionL;
        double tiltPositionR = Constants.tiltIntakePositionR;
        tiltL.setPosition(tiltPositionL);
        tiltR.setPosition(tiltPositionR);

        double dropdownPosition = Constants.dropdownPositionUp;
        dropdown.setPosition(Constants.dropdownPositionUp);

        long timer = System.currentTimeMillis();
        boolean timerRunning = false;

        int liftTarget = 0;
        boolean liftHittingTarget = false;

        boolean depositing = false;

        boolean tiltDropping = false;
        tiltL.setPosition(Constants.tiltIntakePositionL);
        tiltR.setPosition(Constants.tiltIntakePositionR);
        airplane.setPosition(airplaneClosedPosition);

        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;

        boolean blockerOpen = true;
        blocker.setPosition(Constants.blockerOpenPosition);

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

            double triggerSum = g2.right_trigger - g2.left_trigger;

            intakeR.setPower(-triggerSum);
            intakeL.setPower(triggerSum);

            roller.setPower(triggerSum);

            boolean rightBumper = g2.right_bumper;
            if (rightBumper && !lastRightBumper) {
                if (blockerOpen) {
                    blocker.setPosition(Constants.blockerClosedPosition);
                } else {
                    blocker.setPosition(Constants.blockerOpenPosition);
                }
                blockerOpen = !blockerOpen;

            }
            lastRightBumper = rightBumper;

            boolean leftBumper = g2.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                if (tiltDropping) {
                    tiltPositionL = (Constants.tiltIntakePositionL);
                    tiltPositionR = (Constants.tiltIntakePositionR);
                } else {
                    tiltPositionL = (Constants.tiltDropPositionL);
                    tiltPositionR = (Constants.tiltDropPositionR);
                }
                tiltDropping = !tiltDropping;
            }

            boolean triggers = g2.right_trigger > 0.8 && g2.left_trigger > 0.8;

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
            else if (triggers){
                if (!liftHittingTarget)
                    liftTarget = -(liftR.getCurrentPosition());
                liftHittingTarget = true;
                depositing = false;
            }
            else if (g2.cross) {
                liftTarget = 0;
                liftHittingTarget = true;
                depositing = false;
            }

            if ((liftHittingTarget && (Math.abs(liftTarget - liftL.getCurrentPosition()) > 20) || triggers) && !g2.right_stick_button) {
                Constants.setLift(liftTarget, 1);
                if (depositing){
                    blocker.setPosition(Constants.blockerClosedPosition);
                    blockerOpen = false;

                    if (!timerRunning) {
                        timer = System.currentTimeMillis();
                        timerRunning = true;
                    }
                }
                else {
                    blocker.setPosition(Constants.blockerOpenPosition);
                    blockerOpen = true;

                    tiltPositionL = (Constants.tiltIntakePositionL);
                    tiltPositionR = (Constants.tiltIntakePositionR);
                }
            } else {
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                liftL.setPower(g2.left_stick_y);
                liftR.setPower(-g2.left_stick_y);

                liftHittingTarget = false;
            }

            if (timerRunning && System.currentTimeMillis() - timer > 300){
                tiltPositionL = (Constants.tiltDropPositionL);
                tiltPositionR = (Constants.tiltDropPositionR);
                timerRunning = false;
            }

            if(g1.a){
                airplane.setPosition(airplaneOpenPosition);
            }
            else{
                airplane.setPosition(airplaneClosedPosition);
            }

            if (g2.dpad_up){
                dropdownPosition = Constants.dropdownPositionUp;
            }
            if (g2.dpad_down){
                dropdownPosition = Constants.dropdownPositionDown;
            }

            if (g2.dpad_right){
                dropdownPosition -= 0.001;
            }
            if (g2.dpad_left){
                dropdownPosition += 0.001;
            }

            double tiltMultiplier = (Constants.tiltDropPositionR - Constants.tiltIntakePositionR) / (Constants.tiltIntakePositionL - Constants.tiltDropPositionL);

            tiltPositionL -= 0.001 * g2.right_stick_y;
            tiltPositionR += 0.001 * g2.right_stick_y * tiltMultiplier;

            if (tiltPositionL > Constants.tiltDropPositionL) tiltPositionL = Constants.tiltDropPositionL;
            if (tiltPositionL < Constants.tiltIntakePositionL) tiltPositionL = Constants.tiltIntakePositionL;
            if (tiltPositionR > Constants.tiltIntakePositionR) tiltPositionR = Constants.tiltIntakePositionR;
            if (tiltPositionR < Constants.tiltDropPositionR) tiltPositionR = Constants.tiltDropPositionR;

            tiltL.setPosition(tiltPositionL);
            tiltR.setPosition(tiltPositionR);

            if (g2.touchpad){
                liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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