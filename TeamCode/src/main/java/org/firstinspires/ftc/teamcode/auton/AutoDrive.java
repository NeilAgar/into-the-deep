package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

class AutoDrive {
    enum ShoulderLevel {
        SPECIMEN
    }
    public Follower follower;
    DcMotor leftWorm;
    DcMotor rightWorm;
    private Telemetry telemetry;

    public AutoDrive(HardwareMap hardwareMap, Telemetry t) {
        follower = new Follower(hardwareMap);
        leftWorm = hardwareMap.dcMotor.get("leftWorm");
        rightWorm = hardwareMap.dcMotor.get("rightWorm");
        telemetry = t;
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
                updateTelemetry();

                return follower.isBusy();
            }
        };
    }

    public void updateFollower() {
        follower.update();
    }

    public void updateTelemetry() {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData( "heading", follower.getPose().getHeading());
        telemetry.addData( "turn direction", MathFunctions.getTurnDirection(0, Math.PI * 1.5));
        telemetry.update();
    }

    public Action rotateShoulder(ShoulderLevel shoulderLevel) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (shoulderLevel == ShoulderLevel.SPECIMEN) {
                    if (timer.milliseconds() < 1000) {
                        leftWorm.setPower(0.5);
                        rightWorm.setPower(0.5);
                        return true;
                    } else {
                        leftWorm.setPower(0);
                        rightWorm.setPower(0);
                        return false;
                    }
                }

                return false;
            }
        };
    }
}
