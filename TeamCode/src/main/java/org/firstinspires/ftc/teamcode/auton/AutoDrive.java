package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;

public class AutoDrive {

    public Follower follower;
    public DcMotor outtakeSlide;
    public DcMotor outtakeSlide2;
    public int outtakeSlideError = 20;
    public Servo outtakeFlipper;

    public Servo intakeExtension;
    public Servo wrist;
    public Servo leftClaw,rightClaw;
    public CRServo leftIntake, rightIntake;
    public ColorSensor colorSensor;

    public AutoDrive(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);

        outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
        outtakeSlide2 = hardwareMap.dcMotor.get("outtakeSlide2");

        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeExtension = hardwareMap.get(Servo.class, "intakeExtension");
        wrist = hardwareMap.get(Servo.class, "wrist");

        outtakeFlipper = hardwareMap.get(Servo.class, "outtakeFlipper");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    // Sample Outtake Actions:

    public Action flipOuttake() {
        return new SequentialAction(
            setServoPos(outtakeFlipper, 0),
            new SleepAction(1),
            setServoPos(outtakeFlipper, 1)
        );
    }

    public Action moveSlide(int pos) {
        return new SequentialAction(
            setServoPos(wrist, 0.1),
            new ParallelAction(
                setOuttakeSlide(pos),
                new SequentialAction(
                    new SleepAction(0.5),
                    setServoPos(wrist, 0)
                )
            )
        );
    }

    public Action setOuttakeSlide(int slideEnc) {
        return new Action() {
            private boolean initialized = false;
            private Timer timer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeSlide.setTargetPosition(slideEnc);
                    outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (slideEnc > outtakeSlide.getCurrentPosition()) {
                        outtakeSlide2.setPower(1);
                        outtakeSlide.setPower(1);
                    } else {
                        outtakeSlide2.setPower(-1);
                        outtakeSlide.setPower(-1);
                    }

                    timer = new Timer();
                    initialized = true;
                }

                packet.put("slideEnc", slideEnc);
                packet.put("current slide position", outtakeSlide.getCurrentPosition());
                packet.put("slide timer (ms)", timer.getElapsedTime());
                if (timer.getElapsedTime() > 6500 ||
                        Math.abs(outtakeSlide.getCurrentPosition() - slideEnc) < outtakeSlideError) {
                    outtakeSlide.setPower(0);
                    outtakeSlide2.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    public Action resetOuttakeSlide() {
        return packet -> {
            outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return false;
        };
    }

    // Sample Intake Actions:

    public Action intake() {
        return new SequentialAction(
            setServoPos(wrist, 0.84),
            new SleepAction(0.5),
            colorSensingIntake(false)
        );
    }

    public Action transfer() {
        return new SequentialAction(
            setServoPos(wrist, 0),
            new SleepAction(0.5),
            colorSensingIntake(true)
        );
    }

    public Action colorSensingIntake(boolean isTransferring) {
        return new Action() {
            private boolean initialized = false;
            private Timer timer = new Timer();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.resetTimer();
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                    initialized = true;
                }

                if (isTransferring ? (colorSensor.alpha() < 125 || timer.getElapsedTimeSeconds() > 2)
                        : (colorSensor.alpha() > 125 || timer.getElapsedTimeSeconds() > 5)) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    // Specimen Claw Action: (Close - 0.65, 0.2 || Open - 0.3, 0.53)
    public Action moveClaw(double leftClawPos, double rightClawPos) {
        return new SequentialAction(
            setServoPos(leftClaw, leftClawPos),
            setServoPos(rightClaw, rightClawPos)
        );
    }

    // Specimen Cycle Action:
    public Action specimenCycle(PathChain barCycle, PathChain collectCycle, boolean isFinalCycle) {
        return new SequentialAction(
            new ParallelAction(
                followPath(barCycle),
                moveSlide(-1900)
            ),
            new ParallelAction(
                moveSlide(isFinalCycle ? 0 : -135),
                new SequentialAction(
                    new SleepAction(0.5),
                    moveClaw(0.3, 0.53)
                )
            ),
            followPath(collectCycle),
            new SleepAction(0.5),
            moveClaw(isFinalCycle ? 0.3 : 0.65, isFinalCycle ? 0.53 : 0.2)
        );
    }

    // Servo Helper Action:
    public Action setServoPos(Servo servo, double pos) {
        return packet -> {
            servo.setPosition(pos);
            return false;
        };
    }

    // Follower Actions:

    public Action followPath(PathChain path) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    follower.followPath(path);
                    initialized = true;
                }

                updateFollower();
                follower.getDashboardPoseTracker().update();
                updateFollowerTelemetry(packet);

                return follower.isBusy();
            }
        };
    }

    public void updateFollower() {
        follower.update();
    }

    public void updateFollowerTelemetry(TelemetryPacket packet) {
        packet.put("x", follower.getPose().getX());
        packet.put("y", follower.getPose().getY());
        packet.put("heading", follower.getPose().getHeading());
        packet.put("turn direction", MathFunctions.getTurnDirection(0, Math.PI * 1.5));
        packet.put("follower busy", follower.isBusy());
    }

    public Action setFollowerMaxPower(double power) {
        return packet -> {
            follower.setMaxPower(power);
            return false;
        };
    }
}
