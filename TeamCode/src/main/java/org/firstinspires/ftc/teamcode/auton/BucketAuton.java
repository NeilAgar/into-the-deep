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

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;


@Config
@Autonomous(name = "Bucket Side Auton",group = "Autonomous")
public final class BucketAuton extends LinearOpMode {
    AutoDrive drive;

    MultipleTelemetry telemetryA;

    PathChain bucketInit, sampleOne, bucketOne, sampleTwo, bucketTwo, park;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new AutoDrive(hardwareMap);

        drive.intakeExtension.setPosition(1);
        drive.wrist.setPosition(0);
        drive.outtakeFlipper.setPosition(1);

        Pose start = new Pose(0, 0); // 137, 36
        Pose bucket = new Pose(6.7, 21.3);
        Pose sampleOnePos = new Pose(21, 14.25);
        Pose bucketOnePos = new Pose(6.7, 21.3);
        Pose sampleTwoPos = new Pose(16.35, 16.55);
        Pose bucketTwoPos = new Pose(14.55, 18.75);

        drive.follower.setStartingPose(new Pose(0, 0, 0));

        bucketInit = drive.follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(start),
                        new Point(29, 0, Point.CARTESIAN),
                        new Point(bucket)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        sampleOne = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucket),
                        new Point(sampleOnePos)
                ))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(0)
                .build();

        bucketOne = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sampleOnePos),
                        new Point(bucketOnePos)
                ))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();

        sampleTwo = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketOnePos),
                        new Point(sampleTwoPos)
                ))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(0)
                .build();

        bucketTwo = drive.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sampleTwoPos),
                        new Point(bucketTwoPos)
                ))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();

        park = drive.follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketOnePos),
                        new Point(14.65, 0, Point.CARTESIAN),
                        new Point(50, 12, Point.CARTESIAN),
                        new Point(50, -16, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        telemetryA.addLine("Ready to start!");
        telemetryA.update();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
            drive.resetOuttakeSlide(),
            drive.setFollowerMaxPower(0.6),

            new ParallelAction(
                drive.followPath(bucketInit),
                drive.moveSlide(-3100)
            ),
            drive.flipOuttake(),

            new ParallelAction(
                new SequentialAction(
                    drive.followPath(sampleOne),
                    drive.intake()
                ),
                drive.moveSlide(0)
            ),
            drive.transfer(),

            new ParallelAction(
                drive.followPath(bucketOne),
                drive.moveSlide(-3100)
            ),
            drive.flipOuttake(),

            new ParallelAction(
                new SequentialAction(
                    drive.setFollowerMaxPower(0.5),
                    drive.followPath(park)
                ),
                drive.moveSlide(0),
                new SequentialAction(
                    drive.setServoPos(drive.wrist, 1),
                    drive.setServoPos(drive.outtakeFlipper, 0),
                    new SleepAction(0.5),
                    drive.setServoPos(drive.wrist, 0)
                )
            ),
            new SleepAction(30)
        ));
    }
}