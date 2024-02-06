package org.firstinspires.ftc.teamcode; //Importing Libraries

import com.acmerobotics.roadrunner.geometry.Pose2d; //Importing Libraries
import com.qualcomm.robotcore.hardware.CRServo; //Importing Libraries
import com.qualcomm.robotcore.hardware.DcMotor; //Importing Libraries
import com.qualcomm.robotcore.hardware.HardwareMap; //Importing Libraries
import com.qualcomm.robotcore.hardware.Servo; //Importing Libraries

public class Constants { //Declaring Class


    public static DcMotor liftL; // Right Lift
    public static DcMotor liftR; // Left Lift

    public static DcMotor intakeR; //Intake Right

    public static DcMotor intakeL; //Intake Left

    public static Servo blocker;
    public static Servo tiltL; // Outtake Left Side
    public static Servo tiltR; // Outtake Right Side
    public static CRServo roller; //Continuously Rotating Roller

    public static Servo airplane;
    public static Servo dropdown; // Dropdown intake

    public static double tiltDropPositionL = 0.86;//intake tilt position servo Left
    public static double tiltDropPositionR = 0.64;//intake tilt position servo Right

    public static double tiltIntakePositionL = 0.55;//drop tilt position servo Left
    public static double tiltIntakePositionR = 0.95;//drop tilt position servo Right

    public static double blockerOpenPosition = 0.5;
    public static double blockerClosedPosition = 1;

    public static double airplaneOpenPosition = 1;
    public static double airplaneClosedPosition = 0;

    public static double dropdownPositionUp = 0;
    public static double dropdownPositionDown = 0.533;

    public static double dropdownautonpositionstart = 0.385; // Dropdown at start of autonomous

    //BLUE LEFT
    public static Pose2d startPoseBL = new Pose2d(60, -6, Math.toRadians(180)); // Line to Linear Heading - Goes to position (60,-6) while turning 180 degrees
    public static Pose2d purplepixelcenterBL = new Pose2d(31, -6, Math.toRadians(180));
    public static Pose2d purplepixelleftBL = new Pose2d(37, -19, Math.toRadians(180));
    public static Pose2d purplepixelrightBL = new Pose2d(37,3,Math.toRadians(130));
    public static Pose2d purplepixelcenterBLoffset = new Pose2d(36, -6, Math.toRadians(90));
    public static Pose2d purplepixelleftBLoffset = new Pose2d(purplepixelleftBL.getX(),-24,Math.toRadians(180)); // Retrieves the x-coordinate of purplepixelleftBL with y-coordinate of -24, turns 180 degrees
    public static Pose2d purplepixelrightBLoffset = new Pose2d(40,-6,Math.toRadians(180));
    public static Pose2d yellowpixelcenterBL = new Pose2d(35, -34.5, Math.toRadians(90));
    public static Pose2d yellowpixelleftBL = new Pose2d(39,-33,Math.toRadians(90));
    public static Pose2d yellowpixelrightBL = new Pose2d(28,-33.5,Math.toRadians(90));

    public static Pose2d cycledepositBL = new Pose2d(36,-35,Math.toRadians(90));

    public static Pose2d whitepixelBlue = new Pose2d(13, -10, Math.toRadians(90));
    public static Pose2d whitepixelBlue2 = new Pose2d(13, -50, Math.toRadians(90));

    public static Pose2d whitepixeloffsetBlueLeft = new Pose2d(13, 76, Math.toRadians(90));

    public static Pose2d whitepixeloffsetBlueRight = new Pose2d(11.5, 72.5, Math.toRadians(90));



    // BLUE RIGHT
    public static Pose2d startPoseBR = new Pose2d(60, 43, Math.toRadians(180));

    public static Pose2d purplepixelcenterBR = new Pose2d(26, 55, Math.toRadians(0)); //Strafes to (26, 55)

    public static Pose2d purplepixelleftBRoffset = new Pose2d(43, 40, Math.toRadians(-90));
    public static Pose2d purplepixelleftBR = new Pose2d(43, 47, Math.toRadians(-90));

    public static Pose2d purplepixelrightBR = new Pose2d(30,65,Math.toRadians(0));
    public static Pose2d purplepixelBRoffset = new Pose2d(12, 55, Math.toRadians(90));



    public static Pose2d yellowpixelcenterBR = new Pose2d(47, -35, Math.toRadians(90));

    public static Pose2d yellowpixelleftBR = new Pose2d(51,-33,Math.toRadians(90));

    public static Pose2d yellowpixelrightBR = new Pose2d(41,-34,Math.toRadians(90));


    //RED RIGHT

    public static Pose2d startPoseRR = new Pose2d(-60, -1, Math.toRadians(0));
    public static Pose2d purplepixelcenterRR = new Pose2d(-30, -1, Math.toRadians(0));
    public static Pose2d purplepixelrightRR = new Pose2d(-34, -8, Math.toRadians(0));
    public static Pose2d purplepixelleftRR = new Pose2d(-43,3,Math.toRadians(90));


    public static Pose2d purplepixelcenterRRoffset = new Pose2d(-38, -6, Math.toRadians(90));

    public static Pose2d purplepixelrightRRoffset = new Pose2d(-37,-10,Math.toRadians(0));

    public static Pose2d purplepixelleftRRoffset = new Pose2d(-40,-7,Math.toRadians(90));
    public static Pose2d yellowpixelcenterRR = new Pose2d(-44, -41, Math.toRadians(90));
    public static Pose2d yellowpixelrightRR = new Pose2d(-48,-40.5,Math.toRadians(90));

    public static Pose2d yellowpixelrightRRoffset = new Pose2d(-48,-35.5,Math.toRadians(90));
    public static Pose2d yellowpixelleftRR = new Pose2d(-37,-40.5,Math.toRadians(90));
    public static Pose2d parkR = new Pose2d(-36, -33, Math.toRadians(90));
    public static Pose2d parkR2 = new Pose2d(-62, -33, Math.toRadians(90));
    public static Pose2d parkL = new Pose2d(36, -30, Math.toRadians(90));



    //RED LEFT

    public static Pose2d startPoseRL = new Pose2d(-60, 43, Math.toRadians(0));

    public static Pose2d purplepixelcenterRL = new Pose2d(-20, 40, Math.toRadians(-90));

    public static Pose2d purplepixelleftRL = new Pose2d(-12, 54, Math.toRadians(90));

    public static Pose2d purplepixelrightRL = new Pose2d(-37,45,Math.toRadians(-90));

    public static Pose2d purplepixelcenterRLoffset2 = new Pose2d(-12, -6, Math.toRadians(90));
    public static Pose2d purplepixelRLoffset = new Pose2d(-18, 62, Math.toRadians(90));
    public static Pose2d purplepixelleftRLoffset = new Pose2d(purplepixelleftRL.getX(),-24,Math.toRadians(180));

    public static Pose2d purplepixelrightRLoffset = new Pose2d(-37,40,Math.toRadians(-90));

    public static Pose2d yellowpixelcenterRL = new Pose2d(-45, -44, Math.toRadians(90));

    public static Pose2d yellowpixelleftRL = new Pose2d(-41.5,-32,Math.toRadians(90));

    public static Pose2d yellowpixelrightRL = new Pose2d(-28,-32,Math.toRadians(90));

    public static Pose2d whitepixelRed = new Pose2d(-18, -10, Math.toRadians(90));
    public static Pose2d whitepixelRed2 = new Pose2d(-18, -60, Math.toRadians(90));

    public static Pose2d whitepixeloffsetRed = new Pose2d(-20, 68.5, Math.toRadians(90));



    public static int liftTargetAuton = -1500; //Setting position of linear slides - decrease the variable to increase the height
    public static int liftTargetLow = -1600;
    public static int liftTargetMid  = -2000;
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

    public static void setIntake(double power){ //Intake
        intakeL.setPower(power);
        intakeR.setPower(-power);
        roller.setPower(power); //Sets roller to the power variable
    }



    public static void initHardware(HardwareMap hardwareMap){ //Hardware map provides a way to access and configure hardware devices
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
        tiltL = hardwareMap.servo.get("tiltL");
        tiltR = hardwareMap.servo.get("tiltR");
        airplane = hardwareMap.servo.get("airplane");
        dropdown = hardwareMap.servo.get("dropdown");

        roller = hardwareMap.crservo.get("roller");
    }

}
