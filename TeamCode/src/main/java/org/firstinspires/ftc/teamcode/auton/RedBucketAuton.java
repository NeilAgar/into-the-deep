package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Config
@Autonomous(name = "Red Bucket Side Auton",group = "Autonomous")
public final class RedBucketAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoDrive drive = new AutoDrive(hardwareMap, telemetry);

        Pose startingPose = new Pose(132, 36, Math.toRadians(0));
        drive.follower.setStartingPose(startingPose);
        PathChain specimenPath = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                    new Point(startingPose),
                    new Point(118, 72, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
                .build();

        PathChain pickupOne = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(118, 72, Point.CARTESIAN),
                        new Point(120, 24, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        PathChain dropoffOne = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(120, 24, Point.CARTESIAN),
                        new Point(114, 30, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        PathChain pickupTwo = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(114, 30, Point.CARTESIAN),
                        new Point(120, 12, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        PathChain dropoffTwo = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(120, 12, Point.CARTESIAN),
                        new Point(114, 30, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        PathChain park = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(114, 30, Point.CARTESIAN),
                        new Point(84, 24, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
                .addPath(new BezierLine(
                        new Point(84, 24, Point.CARTESIAN),
                        new Point(84, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
            new ParallelAction(
                drive.followPath(specimenPath),
                drive.rotateShoulder(AutoDrive.ShoulderLevel.SPECIMEN)
            ),
            new SleepAction(2),
            new ParallelAction(
                drive.followPath(pickupOne)
            ),
            new SleepAction(2),
            new ParallelAction(
                drive.followPath(dropoffOne),
                drive.rotateShoulder(AutoDrive.ShoulderLevel.SPECIMEN)
            ),
            new SleepAction(2),
            new ParallelAction(
                drive.followPath(pickupTwo)
            ),
            new SleepAction(2),
            new ParallelAction(
                drive.followPath(dropoffTwo)
            ),
            new SleepAction(2),
            new ParallelAction(
                drive.followPath(park)
            )
        ));

//        int pathState = 0;
//
//        while (opModeIsActive()) {
//            telemetry.addData("x", drive.follower.getPose().getX());
//            telemetry.addData("y", drive.follower.getPose().getY());
//            telemetry.addData( "heading", drive.follower.getPose().getHeading());
//            telemetry.addData( "turn direction", MathFunctions.getTurnDirection(0, Math.PI * 1.5));
//            telemetry.update();
//
//            drive.updateFollower();
//
//            switch (pathState) {
//                case 0:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(specimenPath);
//                        drive.rotateShoulder(AutoDrive.ShoulderLevel.SPECIMEN);
//                    }
//                    pathState = 1;
//                    break;
//                case 1:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(pickupOne);
//                    }
//                    pathState = 2;
//                    break;
//                case 2:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(dropoffOne);
//                        drive.rotateShoulder(AutoDrive.ShoulderLevel.SPECIMEN);
//                    }
//                    pathState = 3;
//                    break;
//                case 3:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(pickupTwo);
//                    }
//                    pathState = 4;
//                    break;
//                case 4:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(dropoffTwo);
//                    }
//                    pathState = 5;
//                    break;
//                case 5:
//                    if (!drive.follower.isBusy()) {
//                        drive.follower.followPath(park);
//                    }
//                    pathState = -1;
//                    break;
//            }
//        }
    }
}