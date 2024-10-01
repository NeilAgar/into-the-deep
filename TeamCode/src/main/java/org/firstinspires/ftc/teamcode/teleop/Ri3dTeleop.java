/*
Copyright 2024 FIRST Tech Challenge Team 6547

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop Copy", group = "Teleop")
public class Ri3dTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor leftWorm = hardwareMap.dcMotor.get("leftWorm");
        DcMotor rightWorm = hardwareMap.dcMotor.get("rightWorm");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        // Servo clawServo = hardwareMap.get(Servo.class, "claw");
        // Servo rightClawServo = hardwareMap.get(Servo.class, "rightClaw");
        // Servo leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        Servo leftDiffy = hardwareMap.get(Servo.class, "leftDiffy");
        Servo rightDiffy = hardwareMap.get(Servo.class, "rightDiffy");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        telemetry.addData("leftDiffy", leftDiffy.getPosition());

        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY - rotX - rx) / denominator;
            double backLeftPower = (rotY + rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.right_bumper) {
                leftClaw.setPosition(.9);
                rightClaw.setPosition(.6);
            }
            if (gamepad1.left_bumper) {
                leftClaw.setPosition(.8);
                rightClaw.setPosition(.625);
            }


            if (gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger) {
                rightWorm.setPower(gamepad1.right_trigger);
                leftWorm.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0 && gamepad1.left_trigger > gamepad1.right_trigger) {
                rightWorm.setPower(-gamepad1.left_trigger * .9);
                leftWorm.setPower(-gamepad1.left_trigger * .9);
            }
            else {
                rightWorm.setPower(0);
                leftWorm.setPower(0);
            }


            if (gamepad2.x) {
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.a) {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);
            }
            if (gamepad2.b) {
                slide.setTargetPosition(550);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);
            }
            if (gamepad2.y) {
                slide.setTargetPosition(1250);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);
            }


            // slide.setPower(-gamepad1.right_stick_y);
            // if (gamepad1.y) {
            //     slide.setPower(1);
            // }
            // else if (gamepad1.a) {
            //     slide.setPower(-1);
            // }
            // else {
            //     slide.setPower(0);
            // }


            if (gamepad1.dpad_down) {
                leftDiffy.setPosition(.42);
                rightDiffy.setPosition(.58);
            }
            // if (gamepad1.dpad_left) {
            //     leftDiffy.setPosition(.3);
            //     rightDiffy.setPosition(.7);
            // }
            if (gamepad1.dpad_up) {
                leftDiffy.setPosition(.36);
                rightDiffy.setPosition(.64);
            }
            if (gamepad1.dpad_left) {
                leftDiffy.setPosition(leftDiffy.getPosition() + .0007);
                rightDiffy.setPosition(rightDiffy.getPosition() - .0007);
            }
            if (gamepad1.dpad_right) {
                leftDiffy.setPosition(leftDiffy.getPosition() - .0007);
                rightDiffy.setPosition(rightDiffy.getPosition() + .0007);
            }


            telemetry.addData("leftDiffy", leftDiffy.getPosition());
            telemetry.update();
        }
    }
}