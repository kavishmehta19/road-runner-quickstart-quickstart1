package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {


    public static DcMotor liftL;
    public static DcMotor liftR;

    public static DcMotor intakeR;

    public static DcMotor intakeL;

    public static Servo blocker;
    public static Servo tiltL;
    public static Servo tiltR;
    public static CRServo roller;

    public static Servo airplane;
    public static Servo dropdown;

    public static double tiltIntakePositionL = 0.72;
    public static double tiltIntakePositionR = 0.26;

    public static double tiltDropPositionL = 0.3;
    public static double tiltDropPositionR = 0.65;

    public static double blockerOpenPosition = 0.5;
    public static double blockerClosedPosition = 1;

    public static double airplaneOpenPosition = 1;
    public static double airplaneClosedPosition = 0;

    public static double dropdownPositionUp = 0;
    public static double dropdownPositionDown = 0.533;

    public static double dropdownautonpositionstart = 0.385;

    //BLUE LEFT
    public static Pose2d startPoseBL = new Pose2d(60, -6, Math.toRadians(180));
    public static Pose2d purplepixelcenterBL = new Pose2d(33, -6, Math.toRadians(180));
    public static Pose2d purplepixelleftBL = new Pose2d(40, -18, Math.toRadians(180));
    public static Pose2d purplepixelrightBL = new Pose2d(37,2,Math.toRadians(130));
    public static Pose2d purplepixelcenterBLoffset = new Pose2d(36, -6, Math.toRadians(90));
    public static Pose2d purplepixelleftBLoffset = new Pose2d(purplepixelleftBL.getX(),-24,Math.toRadians(180));
    public static Pose2d purplepixelrightBLoffset = new Pose2d(40,-6,Math.toRadians(180));
    public static Pose2d yellowpixelcenterBL = new Pose2d(35, -34.5, Math.toRadians(90));
    public static Pose2d yellowpixelleftBL = new Pose2d(41,-32.5,Math.toRadians(90));
    public static Pose2d yellowpixelrightBL = new Pose2d(28,-32.5,Math.toRadians(90));

    public static Pose2d cycledepositBL = new Pose2d(36,-35,Math.toRadians(90));

    public static Pose2d whitepixelBlue = new Pose2d(13, -10, Math.toRadians(90));

    public static Pose2d whitepixeloffsetBlue = new Pose2d(13, 75.5, Math.toRadians(90));



    // BLUE RIGHT
    public static Pose2d startPoseBR = new Pose2d(60, 43, Math.toRadians(180));

    public static Pose2d purplepixelcenterBR = new Pose2d(26, 55, Math.toRadians(0));

    public static Pose2d purplepixelleftBR = new Pose2d(12, 54, Math.toRadians(90));

    public static Pose2d purplepixelrightBR = new Pose2d(37,2,Math.toRadians(130));

    public static Pose2d purplepixelcenterBRoffset2 = new Pose2d(12, -6, Math.toRadians(90));
    public static Pose2d purplepixelcenterBRoffset = new Pose2d(12, 74.5, Math.toRadians(90));

    public static Pose2d purplepixelleftBRoffset = new Pose2d(purplepixelleftBL.getX(),-24,Math.toRadians(180));

    public static Pose2d purplepixelrightBRoffset = new Pose2d(40,-6,Math.toRadians(180));

    public static Pose2d yellowpixelcenterBR = new Pose2d(42, -34, Math.toRadians(90));

    public static Pose2d yellowpixelleftBR = new Pose2d(41.5,-32,Math.toRadians(90));

    public static Pose2d yellowpixelrightBR = new Pose2d(28,-32,Math.toRadians(90));


    //RED RIGHT

    public static Pose2d startPoseRR = new Pose2d(-60, -1, Math.toRadians(0));
    public static Pose2d purplepixelcenterRR = new Pose2d(-30, -1, Math.toRadians(0));
    public static Pose2d purplepixelrightRR = new Pose2d(-40, -8, Math.toRadians(0));
    public static Pose2d purplepixelleftRR = new Pose2d(-40,3,Math.toRadians(90));


    public static Pose2d purplepixelcenterRRoffset = new Pose2d(-38, -6, Math.toRadians(90));

    public static Pose2d purplepixelrightRRoffset = new Pose2d(-45,-8,Math.toRadians(0));

    public static Pose2d purplepixelleftRRoffset = new Pose2d(-37,-7,Math.toRadians(90));
    public static Pose2d yellowpixelcenterRR = new Pose2d(-44, -40.5, Math.toRadians(90));
    public static Pose2d yellowpixelrightRR = new Pose2d(-49,-40.5,Math.toRadians(90));
    public static Pose2d yellowpixelleftRR = new Pose2d(-37,-40.5,Math.toRadians(90));
    public static Pose2d parkR = new Pose2d(-36, -33, Math.toRadians(90));
    public static Pose2d parkL = new Pose2d(36, -30, Math.toRadians(90));



    //RED LEFT

    public static Pose2d startPoseRL = new Pose2d(-60, 43, Math.toRadians(0));

    public static Pose2d purplepixelcenterRL = new Pose2d(-26, 55, Math.toRadians(180));

    public static Pose2d purplepixelleftRL = new Pose2d(-12, 54, Math.toRadians(90));

    public static Pose2d purplepixelrightRL = new Pose2d(-37,2,Math.toRadians(130));

    public static Pose2d purplepixelcenterRLoffset2 = new Pose2d(-12, -6, Math.toRadians(90));
    public static Pose2d purplepixelcenterRLoffset = new Pose2d(-12, 55, Math.toRadians(90));

    public static Pose2d purplepixelleftRLoffset = new Pose2d(purplepixelleftRL.getX(),-24,Math.toRadians(180));

    public static Pose2d purplepixelrightRLoffset = new Pose2d(-40,-6,Math.toRadians(180));

    public static Pose2d yellowpixelcenterRL = new Pose2d(-40, -32, Math.toRadians(90));

    public static Pose2d yellowpixelleftRL = new Pose2d(-41.5,-32,Math.toRadians(90));

    public static Pose2d yellowpixelrightRL = new Pose2d(-28,-32,Math.toRadians(90));

    public static Pose2d whitepixelRed = new Pose2d(-20, -10, Math.toRadians(90));

    public static Pose2d whitepixeloffsetRed = new Pose2d(-20, 75.5, Math.toRadians(90));



    public static int liftTargetAuton = -1500;
    public static int liftTargetLow = -1600;
    public static int liftTargetMid  = -2000;
    public static int liftTargetHigh = -2500;

    public static void setLift(int value, double power) {
        // sets both lift motors to the value at the default power
        if (value > liftR.getCurrentPosition())
            power *= -1;

        liftL.setTargetPosition(value);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(power);


        liftR.setTargetPosition(-value);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setPower(-power);
    }

    public static void setIntake(double power){
        intakeL.setPower(power);
        intakeR.setPower(-power);
        roller.setPower(power);
    }



    public static void initHardware(HardwareMap hardwareMap){
        liftL = hardwareMap.dcMotor.get("liftL");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR = hardwareMap.dcMotor.get("intakeR");
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.servo.get("blocker");
        tiltL = hardwareMap.servo.get("tiltL");
        tiltR = hardwareMap.servo.get("tiltR");
        airplane = hardwareMap.servo.get("airplane");
        dropdown = hardwareMap.servo.get("dropdown");

        roller = hardwareMap.crservo.get("roller");
    }

}
