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
@Autonomous(name = "Specimen Side Auton", group = "Autonomous")
public final class SpecimenAuton extends LinearOpMode {
    AutoDrive drive;

    MultipleTelemetry telemetryA;

    PathChain barInit, pushSamples, collectPreload, barCycle, collectCycle;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new AutoDrive(hardwareMap);

        drive.intakeExtension.setPosition(1);
        drive.wrist.setPosition(0);
        drive.outtakeFlipper.setPosition(1);
        drive.rightClaw.setPosition(0.3);
        drive.leftClaw.setPosition(0.53);

        drive.follower.setStartingPose(new Pose(9, 63));

        barInit = drive.follower.pathBuilder()
            .addPath(
                // Line 1
                new BezierCurve(
                    new Point(9.000, 63.000, Point.CARTESIAN),
                    new Point(21.000, 63.000, Point.CARTESIAN),
                    new Point(36.000, 78.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        pushSamples = drive.follower.pathBuilder()
            .addPath(
                // Line 2
                new BezierCurve(
                    new Point(36.000, 78.000, Point.CARTESIAN),
                    new Point(6.000, 18.000, Point.CARTESIAN),
                    new Point(84.000, 40.000, Point.CARTESIAN),
                    new Point(60.000, 24.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                // Line 3
                new BezierLine(
                    new Point(60.000, 24.000, Point.CARTESIAN),
                    new Point(7.000, 24.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                // Line 4
                new BezierCurve(
                    new Point(7.000, 24.000, Point.CARTESIAN),
                    new Point(132.000, 20.000, Point.CARTESIAN),
                    new Point(72.000, 15.000, Point.CARTESIAN),
                    new Point(7.000, 14.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                // Line 5
                new BezierCurve(
                    new Point(7.000, 14.000, Point.CARTESIAN),
                    new Point(132.000, 17.000, Point.CARTESIAN),
                    new Point(72.000, 8.000, Point.CARTESIAN),
                    new Point(7.000, 9.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        collectPreload = drive.follower.pathBuilder()
            .addPath(
                // Line 6
                new BezierCurve(
                    new Point(7.000, 9.000, Point.CARTESIAN),
                    new Point(66.000, 24.000, Point.CARTESIAN),
                    new Point(7.000, 24.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180), 0.5)
            .build();

        barCycle = drive.follower.pathBuilder()
            .addPath(
                // Line 7
                new BezierCurve(
                    new Point(7.000, 24.000, Point.CARTESIAN),
                    new Point(24.000, 24.000, Point.CARTESIAN),
                    new Point(36.000, 90.000, Point.CARTESIAN),
                    new Point(36.000, 64.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0), 0.3)
            .build();

        collectCycle = drive.follower.pathBuilder()
            .addPath(
                // Line 8
                new BezierCurve(
                    new Point(36.000, 64.000, Point.CARTESIAN),
                    new Point(24.000, 67.000, Point.CARTESIAN),
                    new Point(24.000, 24.000, Point.CARTESIAN),
                    new Point(7.000, 24.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180), 0.3)
            .build();

        telemetryA.addLine("Ready to start!");
        telemetryA.update();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
            drive.moveClaw(0.65, 0.2),
            new SleepAction(0.75),

            new ParallelAction(
                new SequentialAction(
                    new SleepAction(1),
                    drive.followPath(barInit)
                ),
                drive.moveSlide(-1900)
            ),
            new ParallelAction(
                drive.moveSlide(-135),
                new SequentialAction(
                    new SleepAction(0.5),
                    drive.moveClaw(0.3, 0.53)
                )
            ),

            drive.followPath(pushSamples),

            drive.followPath(collectPreload),
            new SleepAction(0.5),
            drive.moveClaw(0.65, 0.2),

            drive.specimenCycle(barCycle, collectCycle, false),
            drive.specimenCycle(barCycle, collectCycle, false),
            drive.specimenCycle(barCycle, collectCycle, true)
        ));
    }
}