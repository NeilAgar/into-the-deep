package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static final int OFFSET = 72;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d startingPose = new Pose2d(132 - OFFSET, 36 - OFFSET, Math.toRadians(0));

        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set this to the length and width of your robot
                .setDimensions(17, 17.5)
                // Set this based on your follower constants for PedroPathing
                // (xMovement, yMovement, forwardZeroPowerAcceleration, lateralZeroPowerAcceleration, zeroPowerAccelerationMultiplier)
                .setConstraints(70, -50, -25, -60, 3)
                .build();

        PathChain path = robot.getDrive().pathBuilder()
                .addPath(new BezierLine(
                        new Point(startingPose),
                        new Point(118 - OFFSET, 72 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(118 - OFFSET, 72 - OFFSET, Point.CARTESIAN),
                        new Point(120 - OFFSET, 24 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Point(120 - OFFSET, 24 - OFFSET, Point.CARTESIAN),
                        new Point(114 - OFFSET, 30 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(new BezierLine(
                        new Point(114 - OFFSET, 30 - OFFSET, Point.CARTESIAN),
                        new Point(120 - OFFSET, 12 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Point(120 - OFFSET, 12 - OFFSET, Point.CARTESIAN),
                        new Point(114 - OFFSET, 30 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(new BezierLine(
                        new Point(114 - OFFSET, 30 - OFFSET, Point.CARTESIAN),
                        new Point(84 - OFFSET, 24 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
                .addPath(new BezierLine(
                        new Point(84 - OFFSET, 24 - OFFSET, Point.CARTESIAN),
                        new Point(84 - OFFSET, 42 - OFFSET, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        robot.followPath(path);

        try {
            Image img = ImageIO.read(new File("MeepMeepTesting/src/main/resources/field-2024-juice-dark.png"));
            meepMeep.setBackground(img)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(robot)
                    .start();
        }
        catch (IOException e) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(robot)
                    .start();
        }
    }
}