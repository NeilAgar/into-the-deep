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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

class AutoDrive {

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

    Action intake, outtake, ascent;

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
        rightClaw.setPosition(.3);
        leftClaw.setPosition(.53);
    }

    public Action closeClaw() {
        return packet -> {
            rightClaw.setPosition(.65);
            leftClaw.setPosition(.2);
            return false;
        };
    }

    public Action openClaw() {
        return packet -> {
            rightClaw.setPosition(.3);
            leftClaw.setPosition(.53);
            return false;
        };
    }

    public Action intake() {
        return new SequentialAction(
            setServoPos(wrist, 0.84),
            new SleepAction(0.5),
            colorSensingIntake()
//            spinIntake(1),
//            new SleepAction(0.5),
//            spinIntake(0)
        );
    }

    public Action transfer() {
        return new SequentialAction(
            setServoPos(wrist, 0),
            new SleepAction(0.5),
            spinIntake(1),
            new SleepAction(3),
            spinIntake(0)
        );
    }

    public Action outtake() {
        return new SequentialAction(
            setServoPos(outtakeFlipper, 0),
            new SleepAction(1),
            setServoPos(outtakeFlipper, 1)
        );
    }

    public Action raiseSlide(int pos) {
        return new SequentialAction(
            setServoPos(wrist, 0.1),
            new SleepAction(0.5),
            moveOuttakeSlide(pos),
            setServoPos(wrist, 0)
        );
    }

    public Action dropSlide() {
        return moveOuttakeSlide(0);
    }

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

    public Action moveOuttakeSlide(int slideEnc) {
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

    public Action setServoPos(Servo servo, double pos) {
        return packet -> {
            servo.setPosition(pos);
            return false;
        };
    }

    public Action spinIntake(double power) {
        return packet -> {
            leftIntake.setPower(power);
            rightIntake.setPower(power);
            return false;
        };
    }

    public Action setFollowerMaxPower(double power) {
        return packet -> {
            follower.setMaxPower(power);
            return false;
        };
    }

    public Action colorSensingIntake() {
        return new Action() {
            private boolean initialized = false;
            private Timer timer = new Timer();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized){
                    timer.resetTimer();
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                    initialized = true;
                }
                //false causes action to stop running
                if (colorSensor.alpha() > 125 || timer.getElapsedTimeSeconds() > 5){
                    //sample detected or time exceeds by 5 seconds
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }
}
