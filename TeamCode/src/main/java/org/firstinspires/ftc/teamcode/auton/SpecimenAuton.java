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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Config
@Autonomous(name = "Specimen Side Auton",group = "Autonomous")
public final class SpecimenAuton extends LinearOpMode {
    AutoDrive drive;

    PathChain barInit, specimenOne, firstMoveCloser, moveCloser, barOne, park, firstPickupMid, firstPickupPush, barTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new AutoDrive(hardwareMap);

        drive.intakeExtension.setPosition(1);
        drive.wrist.setPosition(0);
        drive.outtakeFlipper.setPosition(1);
        drive.rightClaw.setPosition(0.3);
        drive.leftClaw.setPosition(0.53);

        Pose start = new Pose(0, 0); // 137, 36
        Pose submersible = new Pose(-26, -8);
        Pose humanPlayerPos = new Pose(-2, 21);
        Pose minutePlayerPos = new Pose(1.25, 21);
        Pose submersibleOne = new Pose(-25.25, -10);
        Pose enterSpecimenArea = new Pose(-48, 31);
        Pose endPushSpecimen = new Pose(-2, 31);
        Pose observation = new Pose(0, 30);
        Pose submersibleTwo = new Pose(-25.25, -14);

        barInit = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(start),
                new Point(-12, 0, Point.CARTESIAN),
                new Point(submersible)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        specimenOne = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(submersibleOne),
                new Point(-8, -8, Point.CARTESIAN),
                new Point(humanPlayerPos)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
            .build();

        moveCloser = drive.follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(humanPlayerPos),
                new Point(minutePlayerPos)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        firstMoveCloser = drive.follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(endPushSpecimen),
                new Point(minutePlayerPos)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        barOne = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(minutePlayerPos),
                new Point(-12, 10, Point.CARTESIAN),
                new Point(submersibleOne)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
            .build();

        firstPickupMid = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(submersible),
                new Point(-12, 18, Point.CARTESIAN),
                new Point(-40, 18, Point.CARTESIAN),
                new Point(enterSpecimenArea)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        firstPickupPush = drive.follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(enterSpecimenArea),
                new Point(endPushSpecimen)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        barTwo = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(minutePlayerPos),
                new Point(-12, 10, Point.CARTESIAN),
                new Point(submersibleTwo)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
            .build();

        park = drive.follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(submersibleTwo),
                new Point(-12, -10, Point.CARTESIAN),
                new Point(observation)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
            drive.closeClaw(),
            new SleepAction(1),

            new ParallelAction(
                new SequentialAction(
                    new SleepAction(1),
                    drive.followPath(barInit)
                ),
                drive.raiseSlide(-2000)
            ),

            new SleepAction(0.5),
            new ParallelAction(
                drive.dropSlide(),
                new SequentialAction(
                    new SleepAction(0.5),
                    drive.openClaw()
                )
            ),

            drive.followPath(firstPickupMid),
            new ParallelAction(
                drive.raiseSlide(-135),
                drive.followPath(firstPickupPush)
            ),
            new SleepAction(0.5),
            new SequentialAction(
                drive.setFollowerMaxPower(0.3),
                drive.followPath(firstMoveCloser)
            ),
            drive.closeClaw(),
            drive.setFollowerMaxPower(1),

            new ParallelAction(
                new SequentialAction(
                    new SleepAction(1),
                    drive.followPath(barOne)
                ),
                drive.raiseSlide(-2000)
            ),

            new SleepAction(0.5),
            new ParallelAction(
                drive.dropSlide(),
                new SequentialAction(
                    new SleepAction(0.5),
                    drive.openClaw()
                )
            ),

            new ParallelAction(
                drive.raiseSlide(-135),
                drive.followPath(specimenOne)
            ),
            new SleepAction(0.5),
            new SequentialAction(
                drive.setFollowerMaxPower(0.3),
                drive.followPath(moveCloser)
            ),
            drive.closeClaw(),
            drive.setFollowerMaxPower(1),

            new ParallelAction(
                new SequentialAction(
                    new SleepAction(1),
                    drive.followPath(barTwo)
                ),
                drive.raiseSlide(-2000)
            ),

            new SleepAction(0.5),
            new ParallelAction(
                drive.dropSlide(),
                new SequentialAction(
                    new SleepAction(0.5),
                    drive.openClaw()
                )
            ),

            drive.followPath(park)
        ));
    }
}