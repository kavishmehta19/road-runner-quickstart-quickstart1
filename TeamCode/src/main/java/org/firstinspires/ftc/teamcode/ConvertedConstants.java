package org.firstinspires.ftc.teamcode; //Importing Libraries

import com.acmerobotics.roadrunner.geometry.Pose2d; //Importing Libraries
import com.qualcomm.robotcore.hardware.CRServo; //Importing Libraries
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor; //Importing Libraries
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap; //Importing Libraries
import com.qualcomm.robotcore.hardware.Servo; //Importing Libraries

public class ConvertedConstants { //Declaring Class

    public static DcMotor liftL; // Right Lift
    public static DcMotor liftR; // Left Lift

    public static DcMotor intakeR; //Intake Right

    public static DcMotor intakeL; //Intake Left

    public static Servo blockerInner;
    public static Servo blocker;
    public static Servo tiltL; // Outtake Left Side
    public static Servo tiltR; // Outtake Right Side
    public static CRServo roller; //Continuously Rotating Roller

    public static ColorSensor colorSensor;

    public static DigitalChannel beamBreak;

    public static Servo airplane;
    public static Servo dropdown; // Dropdown intake

    public static double tiltDropPositionL = 0.867;//intake tilt position servo Left
    public static double tiltDropPositionR = 0.663;//intake tilt position servo Right
    public static double tiltMaxPositionL = 0.96;
    public static double tiltMaxPositionR = 0.57;


    public static double tiltIntakePositionL = 0.52;//drop tilt position servo Left
    public static double tiltIntakePositionR = 0.98;//drop tilt position servo Right

    public static double blockerClosedPosition = 0.5;//hello
    public static double blockerOpenPosition = 0.62;
    public static double blockerWidePosition = 0.75;

    public static double blockerInnerClosedPosition = 0.1;
    public static double blockerInnerOpenPosition = 0.25;

    public static double dropdownPositionUp = 0;
    public static double dropdownPositionDown = .658;

    public static double dropdownIntakeStart = 0.532;
    public static double dropdownPositionAuton = 0.682; // Dropdown at start of autonomous
    public static double dropdownIncrement = 0.03;

    /* BR */
    public static Pose2d startBR = new Pose2d(35, -63, Math.toRadians(90));

    public static Pose2d purpleLeftOffsetBR = new Pose2d(42, -46, Math.toRadians(180));
    public static Pose2d purpleLeftBR = new Pose2d(32, -46, Math.toRadians(180));
    public static Pose2d purpleCenterBR = new Pose2d(47, -29, Math.toRadians(-90)); //Strafes to (26, 55)
    public static Pose2d purpleCloseCenterBR = new Pose2d(37, -35, Math.toRadians(90)); //Strafes to (26, 55)
    public static Pose2d purpleRightBR = new Pose2d(58, -33, Math.toRadians(-90));
    public static Pose2d purpleCloseRightBR = new Pose2d(43, -38, Math.toRadians(90));

    public static Pose2d purpleFirstOffsetBR = new Pose2d(47, -13, Math.toRadians(0));
    public static Pose2d purpleOffsetBR = new Pose2d(47, -15, Math.toRadians(0));
    public static Pose2d purpleCloseOffsetBR = new Pose2d(47, -61.5, Math.toRadians(0));
    public static Pose2d purpleCloseLeftOffsetBR = new Pose2d(55, -62.5, Math.toRadians(0));

    public static Pose2d prepareDropBR = new Pose2d(-28, -19, Math.toRadians(0));
    public static Pose2d prepareDropCloseBR = new Pose2d(-28, -61.5, Math.toRadians(0));
    public static Pose2d prepareDropCloseLeftBR = new Pose2d(-28, -62.5, Math.toRadians(0));

    public static Pose2d yellowCenterBR = new Pose2d(-43, -45.5, Math.toRadians(0));
    public static Pose2d yellowCloseCenterBR = new Pose2d(-43, -35, Math.toRadians(0));
    public static Pose2d yellowLeftBR = new Pose2d(-43, -52.5, Math.toRadians(0));
    public static Pose2d yellowCloseLeftBR = new Pose2d(-43, -48, Math.toRadians(0));
    public static Pose2d yellowRightBR = new Pose2d(-42.5, -39, Math.toRadians(0));
    public static Pose2d yellowCloseRightBR = new Pose2d(-43.5, -28, Math.toRadians(0));
    public static Pose2d yellowResetBR = new Pose2d(-34, -45, Math.toRadians(0));
    public static Pose2d yellowCloseResetBR = new Pose2d(-34, -35, Math.toRadians(0));

    public static Pose2d farParkOffsetBR = new Pose2d(-34, -23, Math.toRadians(0));
    public static Pose2d closeParkOffsetBR = new Pose2d(-34, -59, Math.toRadians(0));
    public static Pose2d closeParkOffsetLeftBR = new Pose2d(-34, -63, Math.toRadians(0));
    public static Pose2d farParkBR = new Pose2d(-54, -23, Math.toRadians(0));
    public static Pose2d closeParkBR = new Pose2d(-54, -59, Math.toRadians(0));
    public static Pose2d closeParkLeftBR = new Pose2d(-54, -63, Math.toRadians(0));
    /* END BR */

    /* BL */public static Pose2d startBL = new Pose2d(-14, -63, Math.toRadians(90)); // Line to Linear Heading - Goes to position (60,-6) while turning 180 degrees

    public static Pose2d purpleCenterBL = new Pose2d(-18, -34.5, Math.toRadians(90));
    public static Pose2d purpleLeftBL = new Pose2d(-29, -39, Math.toRadians(90));
    public static Pose2d purpleRightBL = new Pose2d(-7, -40, Math.toRadians(40));
    public static Pose2d purpleCenterOffsetBL = new Pose2d(-14, -39, Math.toRadians(0));
    public static Pose2d purpleLeftOffsetBL = new Pose2d(-32, -39, Math.toRadians(90)); // Retrieves the x-coordinate of purplepixelleftBL with y-coordinate of -24, turns 180 degrees
    public static Pose2d purpleRightOffsetBL = new Pose2d(-14, -43, Math.toRadians(90));

    public static Pose2d backdropCenterBL = new Pose2d(-44.7, -38, Math.toRadians(0));
    public static Pose2d backdropLeftBL = new Pose2d(-44.7, -42, Math.toRadians(0));
    public static Pose2d backdropRightBL = new Pose2d(-44.7, -31, Math.toRadians(0));

    public static Pose2d crossOffsetBL = new Pose2d(-18, -17, Math.toRadians(0));
    public static Pose2d whitePickupBL = new Pose2d(66, -17, Math.toRadians(0));

    public static Pose2d midParkBL = new Pose2d(-38, -39, Math.toRadians(0));
    public static Pose2d farParkOffsetBL = new Pose2d(-38, -16, Math.toRadians(0));
    public static Pose2d farParkBL = new Pose2d(-36, -16, Math.toRadians(0));
    public static Pose2d closeParkOffsetBL = new Pose2d(-38, -62, Math.toRadians(0));
    public static Pose2d closeParkBL = new Pose2d(-52, -62, Math.toRadians(0));
    /* END BL */

    /* RR */
    public static Pose2d startRR = new Pose2d(10, -63, Math.toRadians(90));
    public static Pose2d purpleCenterRR = new Pose2d(10, -35, Math.toRadians(90));
    public static Pose2d purpleRightRR = new Pose2d(18, -37, Math.toRadians(90));
    public static Pose2d purpleLeftOffsetRR = new Pose2d(16, -43, Math.toRadians(180));
    public static Pose2d purpleLeftRR = new Pose2d(6, -46, Math.toRadians(180));

    public static Pose2d backdropCenterOffsetRR = new Pose2d(15, -47, Math.toRadians(180));
    public static Pose2d purpleCenterOffsetRR = new Pose2d(10, -47, Math.toRadians(90));

    public static Pose2d purpleRightOffsetRR = new Pose2d(19, -40, Math.toRadians(90));

    public static Pose2d backdropCenterRR = new Pose2d(50, -47, Math.toRadians(180));
    public static Pose2d backdropRightRR = new Pose2d(49.5, -51, Math.toRadians(180));
    public static Pose2d backdropLeftRR = new Pose2d(49.5, -40, Math.toRadians(180));

    public static Pose2d crossOffsetRR = new Pose2d(19, -21, Math.toRadians(180));
    public static Pose2d whitePickupRR = new Pose2d(-59.5, -25, Math.toRadians(180));

    public static Pose2d midParkRR = new Pose2d(42, -39, Math.toRadians(180));
    public static Pose2d farParkOffsetRR = new Pose2d(42, -18, Math.toRadians(180));
    public static Pose2d farParkRR = new Pose2d(54, -18, Math.toRadians(180));
    public static Pose2d closeParkOffsetRR = new Pose2d(42, -70, Math.toRadians(180));
    public static Pose2d closeParkRR = new Pose2d(54, -70, Math.toRadians(180));
    /* END RR */


    /* RL */
    public static Pose2d startRL = new Pose2d(-38,-63,  Math.toRadians(90));

    public static Pose2d purpleCenterRL = new Pose2d( -35, -24,Math.toRadians(0));
    public static Pose2d purpleLeftRL = new Pose2d(-35.5, -35, Math.toRadians(-90));
    public static Pose2d purpleLeftOffsetRL = new Pose2d( -55,-23, Math.toRadians(180));
    public static Pose2d purpleRightOffsetRL = new Pose2d(-40, -33, Math.toRadians(0));
    public static Pose2d purpleRightRL = new Pose2d( -25,-33, Math.toRadians(0));
    public static Pose2d purpleOffsetRL = new Pose2d( -57, -21,Math.toRadians(180));

    public static Pose2d prepareDropRL = new Pose2d( 35, -21,Math.toRadians(180));

    public static Pose2d yellowCenterRL = new Pose2d(49.5, -45.5, Math.toRadians(180));
    public static Pose2d yellowLeftRL = new Pose2d(49.5,-39,  Math.toRadians(180));
    public static Pose2d yellowRightRL = new Pose2d( 49.5, -52.5,Math.toRadians(180));
    public static Pose2d yellowOffsetRL = new Pose2d( 41, -45,Math.toRadians(180));

    public static Pose2d farParkOffsetRL = new Pose2d(41, -21, Math.toRadians(180));
    public static Pose2d farParkRL = new Pose2d( 58,-21, Math.toRadians(180));
    /* END RL */


    public static int liftTargetAuton = -1500; //Setting position of linear slides - decrease the variable to increase the height
    public static int liftTargetLow = -1600;
    public static int liftTargetMid = -2000;
    public static int liftTargetHigh = -2500;

    public static void setLift(int value, double power) { //Linear Slides
        // sets both lift motors to the value at the default power
        if (value > liftR.getCurrentPosition()) //If the value is greater than the current position of the LiftR Motor, it multiplies the power by -1
            power *= -1;

        liftL.setTargetPosition(value); //Sets LiftL target position to value
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Motor will keep running until it reaches Target Position
        liftL.setPower(power); //Sets the power to the power variable

        //MIRROR IMAGE OF LiftL
        liftR.setTargetPosition(-value);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setPower(-power);
    }

    public static void setIntake(double power) { //Intake
        intakeL.setPower(power);
        intakeR.setPower(-power);
        roller.setPower(power); //Sets roller to the power variable
    }


    public static void initHardware(HardwareMap hardwareMap) { //Hardware map provides a way to access and configure hardware devices
        liftL = hardwareMap.dcMotor.get("liftL"); //Initializes LiftL Motor by getting it from the HardwareMap
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Brakes the motor for LiftL
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops and resets the encoder's power to 0; encoders measure the motor's position
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // uses feedback from encoder to control and maintain a specific speed

        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR = hardwareMap.dcMotor.get("intakeR");
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.servo.get("blocker");
        blockerInner = hardwareMap.servo.get("blocker2");
        tiltL = hardwareMap.servo.get("tiltL");
        tiltR = hardwareMap.servo.get("tiltR");
        airplane = hardwareMap.servo.get("airplane");
        dropdown = hardwareMap.servo.get("dropdown");

        roller = hardwareMap.crservo.get("roller");

        colorSensor = hardwareMap.colorSensor.get("color");

        beamBreak = hardwareMap.get(DigitalChannel.class, "beam1");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
    }

}
