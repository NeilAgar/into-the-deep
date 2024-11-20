package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

import com.acmerobotics.dashboard.config.Config;

@Config
@Disabled
@TeleOp(name = "Drive Enhanced Teleop (Android Studio)", group = "Teleop")
public class TeleOpEnhancements extends LinearOpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public static int slideEnc = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower.startTeleopDrive();

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        DcMotor outtakeSlide = hardwareMap.dcMotor.get("leftWorm");

        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        Servo leftDiffy = hardwareMap.get(Servo.class, "leftDiffy");
        Servo rightDiffy = hardwareMap.get(Servo.class, "rightDiffy");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.options) {
                imu.resetYaw();
            }

            outtakeSlide.setTargetPosition(slideEnc);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(0.4);

            if (gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger) {
                slideEnc += slideEnc < 500 ? 30 : 0;
            }
            else if (gamepad1.left_trigger > 0 && gamepad1.left_trigger > gamepad1.right_trigger) {
                slideEnc -= slideEnc > 0 ? 30 : 0;
            }

            if (gamepad1.x || gamepad2.x) {
                outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.right_bumper) {
                leftClaw.setPosition(.215);
                rightClaw.setPosition(.92);
            }
            if (gamepad1.left_bumper) {
                leftClaw.setPosition(.5);
                rightClaw.setPosition(.65);
            }

            if (gamepad1.dpad_down) {
                leftDiffy.setPosition(0.55);
                rightDiffy.setPosition(0.45);
            }
            if (gamepad1.dpad_right) {
                leftDiffy.setPosition(.5);
                rightDiffy.setPosition(.5);
            }
            if (gamepad1.dpad_left) {
                leftDiffy.setPosition(.54);
                rightDiffy.setPosition(.46);
            }
            if (gamepad1.dpad_up/* || gamepad1.right_stick_button*/) {
                leftDiffy.setPosition(.4805555555556);
                rightDiffy.setPosition(.6583333333);
            }
            if (gamepad1.left_stick_button) {
                leftDiffy.setPosition(.58);
                rightDiffy.setPosition(.42);
            }

            if (gamepad2.dpad_up) {
                leftDiffy.setPosition(leftDiffy.getPosition() + .001);
                rightDiffy.setPosition(rightDiffy.getPosition() - .001);
            }
            if (gamepad2.dpad_down) {
                leftDiffy.setPosition(leftDiffy.getPosition() - .001);
                rightDiffy.setPosition(rightDiffy.getPosition() + .001);
            }
            if (gamepad2.dpad_right) {
                leftDiffy.setPosition(leftDiffy.getPosition() + .001);
                rightDiffy.setPosition(rightDiffy.getPosition() + .001);
            }
            if (gamepad2.dpad_left) {
                leftDiffy.setPosition(leftDiffy.getPosition() - .001);
                rightDiffy.setPosition(rightDiffy.getPosition() - .001);
            }

            telemetry.addData("leftDiffy", leftDiffy.getPosition());
            telemetry.addData("rightDiffy", rightDiffy.getPosition());
            telemetry.update();
        }
    }
}
