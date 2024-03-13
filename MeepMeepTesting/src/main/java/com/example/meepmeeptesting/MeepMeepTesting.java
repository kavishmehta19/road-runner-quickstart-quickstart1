package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d startBR = new Pose2d(35, -63, Math.toRadians(90));

    public static Pose2d purpleLeftOffsetBR = new Pose2d(42, -36, Math.toRadians(180));
    public static Pose2d purpleLeftBR = new Pose2d(32, -36, Math.toRadians(180));
    public static Pose2d purpleCenterOffsetBR = new Pose2d(52, -29, Math.toRadians(180)); //Strafes to (26, 55)
    public static Pose2d purpleCenterBR = new Pose2d(48, -29, Math.toRadians(180)); //Strafes to (26, 55)
    public static Pose2d purpleCloseCenterBR = new Pose2d(37, -35, Math.toRadians(90)); //Strafes to (26, 55)
    public static Pose2d purpleRightFirstOffsetBR = new Pose2d(36,-12,Math.toRadians(90));
    public static Pose2d purpleRightSecondOffsetBR = new Pose2d(55,-12,Math.toRadians(-90));
    public static Pose2d purpleRightBR = new Pose2d(55, -20, Math.toRadians(-90));
    public static Pose2d purpleRightCloseBR = new Pose2d(43, -43, Math.toRadians(90));
    public static Pose2d purpleRightCloseOffsetBR = new Pose2d(43, -50, Math.toRadians(90));

    public static Pose2d purpleFirstOffsetBR = new Pose2d(47, -13, Math.toRadians(0));
    public static Pose2d purpleOffsetBR = new Pose2d(52, -12, Math.toRadians(0));
    public static Pose2d farIntakeBR = new Pose2d(60,-12,Math.toRadians(0));
    public static Pose2d purpleCloseOffsetBR = new Pose2d(47, -61.5, Math.toRadians(0));
    public static Pose2d purpleCloseLeftOffsetBR = new Pose2d(55, -62.5, Math.toRadians(0));

    public static Pose2d closeIntakeOffsetBR = new Pose2d(58,-36,Math.toRadians(0));
    public static Pose2d closeIntakeBR = new Pose2d(63,-36,Math.toRadians(0));

    public static Pose2d prepareDropBR = new Pose2d(-28, -12, Math.toRadians(0));
    public static Pose2d prepareDropCloseBR = new Pose2d(-28, -61.5, Math.toRadians(0));
    public static Pose2d prepareDropCloseLeftBR = new Pose2d(-28, -62.5, Math.toRadians(0));

    public static Pose2d yellowCenterBR = new Pose2d(-53, -36, Math.toRadians(0));
    public static Pose2d yellowCloseCenterBR = new Pose2d(-53, -26, Math.toRadians(0));
    public static Pose2d yellowLeftBR = new Pose2d(-53, -43, Math.toRadians(0));
    public static Pose2d yellowCloseLeftBR = new Pose2d(-53, -39, Math.toRadians(0));
    public static Pose2d yellowRightBR = new Pose2d(-53, -30, Math.toRadians(0));
    public static Pose2d yellowCloseRightBR = new Pose2d(-53, -19, Math.toRadians(0));
    public static Pose2d yellowResetBR = new Pose2d(-42, -36, Math.toRadians(0));
    public static Pose2d yellowCloseResetBR = new Pose2d(-42, -26, Math.toRadians(0));

    public static Pose2d farParkOffsetBR = new Pose2d(-42, -12, Math.toRadians(0));
    public static Pose2d closeParkOffsetBR = new Pose2d(-34, -59, Math.toRadians(0));
    public static Pose2d closeParkOffsetLeftBR = new Pose2d(-34, -63, Math.toRadians(0));
    public static Pose2d farParkBR = new Pose2d(-54, -12, Math.toRadians(0));
    public static Pose2d closeParkBR = new Pose2d(-54, -59, Math.toRadians(0));
    public static Pose2d closeParkLeftBR = new Pose2d(-54, -63, Math.toRadians(0));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d pose1 = new Pose2d(35, -63, Math.toRadians(90));
        Pose2d pose2 = new Pose2d(42, -46, Math.toRadians(180));

        Pose2d pose3 = new Pose2d(60, 43, Math.toRadians(180));
        Pose2d pose4 = new Pose2d(43, 50, Math.toRadians(-90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startBR)
                        .lineToLinearHeading(purpleRightCloseOffsetBR)
                        .lineToLinearHeading(purpleRightCloseBR)
                        .lineToLinearHeading(purpleRightCloseOffsetBR)
                        .lineToLinearHeading(closeIntakeOffsetBR)
                        .lineToLinearHeading(closeIntakeBR)
                                .lineToLinearHeading(closeIntakeOffsetBR)
                        .lineToLinearHeading(purpleCloseOffsetBR)
                        .lineToLinearHeading(prepareDropCloseBR)
                        .lineToLinearHeading(yellowResetBR)
                        .lineToLinearHeading(yellowCenterBR)
                        .lineToLinearHeading(yellowResetBR)
                        .lineToLinearHeading(closeParkOffsetBR)
                        .lineToLinearHeading(closeParkBR)
                        .build()
                );

//                                .lineToLinearHeading(purpleLeftOffsetRR)
//                                .lineToLinearHeading(purpleLeftRR)
//                                .lineToLinearHeading(purpleLeftOffsetRR)
//                                .lineToLinearHeading(midParkRR)
//                                .lineToLinearHeading(backdropCenterRR)
//                                .lineToLinearHeading(midParkRR)
//                                .lineToLinearHeading(crossOffsetRR)
//                                .lineToLinearHeading(whitePickupRR)
//                                .lineToLinearHeading(crossOffsetRR)
//                                .lineToLinearHeading(midParkRR)
//                                .lineToLinearHeading(backdropLeftRR)
//                                .lineToLinearHeading(midParkRR)
//                                .build()

//                                  .lineToLinearHeading(purpleLeftOffsetBL)
//                                .lineToLinearHeading(purpleLeftBL)
//                                .lineToLinearHeading(purpleLeftOffsetBL)
//                                .lineToLinearHeading(midParkBL)
//                                .lineToLinearHeading(backdropLeftBL)
//                                .lineToLinearHeading(midParkBL)
//                                .lineToLinearHeading(crossOffsetBL)
//                                .lineToLinearHeading(whitePickupBL)
//                                .lineToLinearHeading(crossOffsetBL)
//                                .lineToLinearHeading(midParkBL)
//                                .lineToLinearHeading(backdropLeftBL)
//                                .lineToLinearHeading(midParkBL)
//                                .build()

//        drive.trajectorySequenceBuilder(startBR)
//                .lineToLinearHeading(purpleRightFirstOffsetBR)
//                .lineToLinearHeading(purpleRightSecondOffsetBR)
//                .lineToLinearHeading(purpleRightBR)
//                .lineToLinearHeading(purpleRightSecondOffsetBR)
//                .lineToLinearHeading(purpleOffsetBR)
//                .lineToLinearHeading(farIntakeBR)
//                .lineToLinearHeading(prepareDropBR)
//                .lineToLinearHeading(yellowResetBR)
//                .lineToLinearHeading(yellowRightBR)
//                .lineToLinearHeading(yellowResetBR)
//                .lineToLinearHeading(farParkOffsetBR)
//                .lineToLinearHeading(farParkBR)
//                .build()

//        drive.trajectorySequenceBuilder(startRL)
//                        .lineToLinearHeading(purpleCenterOffsetRL)
//                        .lineToLinearHeading(purpleCenterRL)
//                        .lineToLinearHeading(purpleCenterOffsetRL)
//                        .lineToLinearHeading(purpleOffsetRL)
//                        .lineToLinearHeading(farIntakeRL)
//                        .lineToLinearHeading(prepareDropRL)
//                        .lineToLinearHeading(yellowOffsetRL)
//                        .lineToLinearHeading(yellowCenterRL)
//                        .lineToLinearHeading(yellowOffsetRL)
//                        .lineToLinearHeading(farParkOffsetRL)
//                        .lineToLinearHeading(farParkRL)
//                        .build()

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}