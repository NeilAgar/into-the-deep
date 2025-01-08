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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Dereks TeleOp (Android Studio)")
public class DerekTeleOp extends LinearOpMode {
        // outtake arm encoder value
        public int slideEnc = 0;

        // intake extension position
        public double extensionPos = .91;

        // intake direction
        public double intakeValue = 0;

        public double transfer = 0;

        public double intakePos = .84;
        // toggle button state values
        // public boolean wristUp = false;
        // public boolean wristButtonHeld = false;
        // public boolean flipperUp = false;
        // public boolean flipperButtonHeld = false;
        // public boolean outtakeButtonHeld = false;
        // public boolean intakeButtonHeld = false;
        public boolean slideHome = false;
        public boolean doIntake = false;
        public boolean couldHaveSample = false;
        public boolean holdSample = false;
        public boolean readyToTransfer = false;
        public boolean doTransfer = false;
        public boolean boxSample = false;
        public boolean sampleScored = false;
        public double wristTimer = 31;

        public boolean samples = true;

        public int timer = 0;

        // Motors / Servos
        DcMotor frontLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backLeftMotor;
        DcMotor backRightMotor;

        IMU imu;

        DcMotor outtakeSlide;
        DcMotor outtakeSlide2;
        Servo outtakeFlipper;

        Servo leftClaw;
        Servo rightClaw;

        Servo intakeExtension;
        Servo wrist;
        CRServo leftIntake;
        CRServo rightIntake;
        // ColorRangeSensor distanceSensor;
        ColorSensor colorSensor;


        public void drivetrainLoop(Gamepad driveGamepad) {
                double y = -driveGamepad.left_stick_y; // Remember, Y stick value is reversed
                double x = driveGamepad.left_stick_x;
                double rx = -(driveGamepad.right_stick_x * .8);
                if (driveGamepad.right_stick_y > rx) {
                        rx = 0;
                }

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (driveGamepad.share) {
                        imu.resetYaw();
                        wrist.setPosition(.3);
                        outtakeFlipper.setPosition(.775); // changed from .85 to .95
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +Math.PI;

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = 1 * Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX - rx) / denominator;
                double backLeftPower = (rotY - rotX - rx) / denominator;
                double frontRightPower = (rotY - rotX + rx) / denominator;
                double backRightPower = (rotY + rotX + rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Ignore but don't delete (derek)
                // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

                // Drive Train Motors
                frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
                backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
                frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
                backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

                // Reverse the Left Side of the Drive Train
                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                // Other Devices
                outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
                outtakeSlide2 = hardwareMap.dcMotor.get("outtakeSlide2");
                colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
                // distanceSensor = hardwareMap.get(ColorRangeSensor.class, "distanceSensor");

                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                intakeExtension = hardwareMap.get(Servo.class, "intakeExtension");
                wrist = hardwareMap.get(Servo.class, "wrist");

                outtakeFlipper = hardwareMap.get(Servo.class, "outtakeFlipper");
                // outtakeFlipper.setPosition(.8);
                // outtakeFlipper.setPosition(flipperUp ? 0 : 0.8);
                // flipperUp = !flipperUp;

                rightClaw = hardwareMap.get(Servo.class, "rightClaw");
                leftClaw = hardwareMap.get(Servo.class, "leftClaw");
                rightClaw.setPosition(.3);
                leftClaw.setPosition(.53);

                rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
                leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
                leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

                // Retrieve the IMU from the hardware map
                imu = hardwareMap.get(IMU.class, "imu");
                // Adjust the orientation parameters to match your robot
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
                imu.initialize(parameters);

                waitForStart();

                if (isStopRequested()) return;

                while (opModeIsActive()) {
                        drivetrainLoop(gamepad1);

                        if (gamepad1.left_stick_button) {
                                samples = true;
                        }
                        if (gamepad1.touchpad) {
                                samples = false;
                        }

                        if (samples) {
                                samples();
                        }
                        if (!samples) {
                                specimen();
                        }

                        // telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.CM));
                        telemetry.addData("samples: ", samples);
                        telemetry.addData("doIntake: ", doIntake);
                        telemetry.addData("wrist: ", wrist.getPosition());
                        // telemetry.addData("changed: ", changed);
                        // telemetry.addData("leftIntake: ", leftIntake.getPower());
                        telemetry.addData("timer: ", timer);
                        telemetry.addData("slideHome: ", slideHome);
                        telemetry.addData("couldHaveSample: ", couldHaveSample);
                        telemetry.addData("holdSample: ", holdSample);
                        telemetry.addData("readyToTransfer: ", readyToTransfer);
                        telemetry.addData("doTransfer: ", doTransfer);
                        // telemetry.addData("boxSample: ", boxSample);

                        telemetry.addData("slide 1: ", outtakeSlide.getPower());
                        telemetry.addData("slide 2: ", outtakeSlide2.getPower());
                        telemetry.addData("red: ", colorSensor.red());
                        telemetry.addData("blue: ", colorSensor.blue());
                        telemetry.addData("alpha: ", colorSensor.alpha());

                        telemetry.update();
                }
        }

        public void samples() {
                // // // Trigger Outtake Slide:
                // // // outtakeSlide.setTargetPosition(slideEnc);
                // // // outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // if (gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger) {
                //     outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //     outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //     outtakeSlide.setPower(-gamepad1.right_trigger);
                //     outtakeSlide2.setPower(-gamepad1.right_trigger);
                // }
                // else if (gamepad1.left_trigger > 0 && gamepad1.left_trigger > gamepad1.right_trigger) {
                //     if (outtakeSlide.getCurrentPosition() < -5 || gamepad1.b) {
                //         outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //         outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //         outtakeSlide.setPower(gamepad1.left_trigger);
                //         outtakeSlide2.setPower(gamepad1.left_trigger);
                //     }
                // }


                if (gamepad2.right_trigger > 0 && gamepad2.right_trigger > gamepad1.left_trigger) {
                        //move linear slide up
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        outtakeSlide.setPower(-gamepad2.right_trigger);
                        outtakeSlide2.setPower(-gamepad2.right_trigger);
                }
                else if (gamepad2.right_trigger > 0 && gamepad2.right_trigger > gamepad2.left_trigger) {
                        //also move linear slide up
                        if (outtakeSlide.getCurrentPosition() >= 5 || gamepad2.b) {
                                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                outtakeSlide.setPower(gamepad2.left_trigger);
                                outtakeSlide2.setPower(gamepad2.left_trigger);
                        }
                }
                else if (gamepad1.dpad_right && gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        outtakeSlide.setPower(-gamepad1.right_trigger);
                        outtakeSlide2.setPower(-gamepad1.right_trigger);
                }
                else if (gamepad1.dpad_right && gamepad1.left_trigger > 0) {
                        if (outtakeSlide.getCurrentPosition() < -5 || gamepad1.b) {
                                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                outtakeSlide.setPower(gamepad1.left_trigger);
                                outtakeSlide2.setPower(gamepad1.left_trigger);
                        }
                } else if (gamepad1.right_stick_button) {
                        if (gamepad1.right_stick_y >= .5 || gamepad1.right_stick_y <= -.5) {
                                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                outtakeSlide.setPower(gamepad1.right_stick_y);
                                outtakeSlide2.setPower(gamepad1.right_stick_y);
                        }
                }
                else if (outtakeSlide.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        // if (outtakeSlide.getCurrentPosition() < -10) {
                        //     outtakeSlide.setPower(-1);
                        //     // outtakeSlide2.setPower(-1);
                        // }
                        outtakeSlide.setPower(-1);
                        // outtakeSlide2.setPower(-1);
                }
                else {
                        outtakeSlide.setPower(0);
                        outtakeSlide2.setPower(0);
                }

                if (outtakeSlide.getTargetPosition() - outtakeSlide.getCurrentPosition() <= 15 && outtakeSlide.getCurrentPosition() - outtakeSlide.getTargetPosition() <= 15) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (gamepad1.y || gamepad2.y/* || gamepad1.right_bumper*/) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-3000);
                        outtakeSlide2.setPower(-1);

                        // outtakeFlipper.setPosition(0);
                }
                if (gamepad1.b || gamepad2.b) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-825);
                        if (outtakeSlide.getCurrentPosition() < -1000) {
                                outtakeSlide2.setPower(-1);
                        }
                        else {
                                outtakeSlide2.setPower(1);
                        }
                }
                if (gamepad1.a || gamepad2.a/* || gamepad1.left_bumper*/) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(0);
                        outtakeSlide2.setPower(1);
                        outtakeFlipper.setPosition(.8);
                }



                if (outtakeSlide.getTargetPosition() == outtakeSlide.getCurrentPosition()) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }


                if (gamepad1.dpad_right) {
                        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slideEnc = 0;
                }

                telemetry.addData("flipper", outtakeFlipper.getPosition());

                telemetry.addData("outtakeSlide", outtakeSlide.getCurrentPosition());
                telemetry.addData("outtakeSlide Position", slideEnc);

                // // // Button Intake Slide:
                // // // if (gamepad2.circle) extensionPos = Range.clip(extensionPos - 0.005, 0.7, 1);
                // // // else if (gamepad2.square) extensionPos = Range.clip(extensionPos + 0.005, 0.7, 1);
                // if (gamepad2.a) {
                //     extensionPos = .91;
                // }
                // if (gamepad2.b) {
                //     extensionPos = .85;
                // }
                // if (gamepad2.y) {
                //     extensionPos = .71;
                // }
                // if (gamepad2.right_trigger > 0 && gamepad2.right_trigger > gamepad2.left_trigger && extensionPos > .71) {
                //     extensionPos -= gamepad2.right_trigger * .005;
                // }
                // else if (gamepad2.left_trigger > 0 && extensionPos < .91) {
                //     extensionPos += gamepad2.left_trigger * .005;
                // }
                if (!gamepad1.dpad_right && gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger && extensionPos > .71) {
                        extensionPos -= gamepad1.right_trigger * .005;
                }
                else if (!gamepad1.dpad_right && gamepad1.left_trigger > 0 && extensionPos < .91) {
                        extensionPos += gamepad1.left_trigger * .005;
                }

                intakeExtension.setPosition(extensionPos);

                telemetry.addData("extensionPos", extensionPos);
                telemetry.addData("intake extension position", intakeExtension.getPosition());

                // // // Toggle Wrist:
                // // // if (gamepad2.left_bumper) {
                // // //     if (!wristButtonHeld) {
                // // //         wrist.setPosition(wristUp ? .3 : .56);
                // // //         wristUp = !wristUp;
                // // //         wristButtonHeld = true;
                // // //     }
                // // // }
                if (gamepad2.left_bumper) {
                        wrist.setPosition(transfer);
                        // couldHaveSample = false;
                }
                else if (gamepad2.right_bumper) {
                        wrist.setPosition(intakePos);
                        // couldHaveSample = true;

                }
                // else if (wristButtonHeld) {
                //     wristButtonHeld = false;
                // }
                if (gamepad2.right_stick_y != 0) {
                        wrist.setPosition(wrist.getPosition() + gamepad2.right_stick_y * .01);
                        // couldHaveSample = false;
                }
                else if (!gamepad1.right_stick_button) {
                        if (gamepad1.right_stick_y != 0) {
                                wrist.setPosition(wrist.getPosition() + gamepad1.right_stick_y * .01);
                                // couldHaveSample = false;
                        }
                }



                // // // if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                // // //     wrist.setPosition(Range.clip(wrist.getPosition() + gamepad2.left_stick_x, 0, 1));
                // // // }

                // // // // Toggle Flipper:
                if (gamepad1.dpad_left) {
                        outtakeFlipper.setPosition(.775); // changed from .85 to .95
                }
                if (gamepad1.dpad_up) {
                        outtakeFlipper.setPosition(0);
                }
                // if (gamepad1.dpad_left) {
                //     outtakeFlipper.setPosition(outtakeFlipper.getPosition()+.05);
                // }
                // if (gamepad1.dpad_up) {
                //     outtakeFlipper.setPosition(outtakeFlipper.getPosition()+.05);
                // }


                // // if (!flipperButtonHeld) {
                //     //     outtakeFlipper.setPosition(flipperUp ? 0 : 0.8);
                //     //     flipperUp = !flipperUp;
                //     //     flipperButtonHeld = true;
                //     // }
                // // else if (flipperButtonHeld) {
                // //     flipperButtonHeld = false;
                // // }

                // // if (gamepad1.y) {
                // //     outtakeFlipper.setPosition(flipperUp ? 0 : 0.8);
                // // }
                // // if (gamepad1.a) {
                // //     flipperButtonHeld = false;
                // // }

                // // Toggle Intake:
                // // if (gamepad1.dpad_down) {
                // //     if (!intakeButtonHeld) {
                // //         intakeValue = intakeValue == -0.5 ? 0 : -0.5;
                // //         intakeButtonHeld = true;
                // //     }
                // // }
                // // else if (intakeButtonHeld) {
                // //     intakeButtonHeld = false;
                // // }

                // // if (gamepad1.dpad_up) {
                // //     if (!outtakeButtonHeld) {
                // //         intakeValue = intakeValue == 1 ? 0 : 1;
                // //         outtakeButtonHeld = true;
                // //     }
                // // }
                // // else if (outtakeButtonHeld) {
                // //     outtakeButtonHeld = false;
                // // }

                // // Hold Down Intake:
                // if (!flipperButtonHeld) {
                //     outtakeFlipper.setPosition(flipperUp ? 0 : 0.8);
                //     flipperUp = !flipperUp;
                //     flipperButtonHeld = true;
                // }
                // else if (flipperButtonHeld) {
                //     flipperButtonHeld = false;
                // }
                // if (intakeButtonHeld) {
                //         intakeValue = (doIntake ? 1 : 0);
                //         doIntake = !doIntake;
                //         intakeButtonHeld = true;
                //     }
                // else if (!intakeButtonHeld) {
                //     intakeButtonHeld = false;
                // }
                // if (gamepad2.dpad_up && !changed) {
                //     if (leftIntake.getPower() == -0) leftIntake.setPower(1);
                //     else leftIntake.setPower(0);
                //     changed = true;
                // } else if (!gamepad2.dpad_up) {
                //     changed = false;
                // }
                if (gamepad2.dpad_down || gamepad1.left_bumper) {
                        intakeValue = -1;
                        doIntake = false;
                }else if (doIntake || doTransfer || gamepad2.dpad_right) {
                        intakeValue = 1;
                }else{
                        intakeValue = 0;
                }
                if (gamepad2.dpad_up) {
                        doIntake = true;
                }
                if (gamepad1.right_bumper) { // begins intake
                        doIntake = true;
                        wristTimer = 0;
                        wrist.setPosition(intakePos);
                        // couldHaveSample = true;
                }
                if (gamepad1.dpad_down) {
                        doIntake = false;
                        intakeValue = 1;
                }
                wristTimer++; // (idk i'm making a timer)
                if (wristTimer == 15) { // after enough time it enables having a sample
                        couldHaveSample = true;
                }
                if (colorSensor.alpha() >= 125 && couldHaveSample) { // detects sample and stops intaking
                        doIntake = false;
                        holdSample = true;
                        couldHaveSample = false;
                }
                if (holdSample) { // moves to transfer and starts timer
                        wrist.setPosition(transfer);
                        extensionPos = .91;
                        readyToTransfer = true;
                        timer++;
                }
                if (holdSample && readyToTransfer && timer > 50) { // after enough time transfers
                        doTransfer = true;
                }
                if (doTransfer && colorSensor.alpha() < 250) { // detects once sample is gone and stops transferring + moves wrist out of way
                        doTransfer = false;
                        holdSample = false;
                        boxSample = true;
                        wrist.setPosition(.1);
                }
                if (boxSample) { // moves to outtake position
                        //     outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //     outtakeSlide.setTargetPosition(-3000);
                        //     boxSample = false;
                }


                if (gamepad1.ps) {
                        slideHome = false;
                        doIntake = false;
                        couldHaveSample = false;
                        holdSample = false;
                        readyToTransfer = false;
                        doTransfer = false;
                        boxSample = false;
                        sampleScored = false;
                        wristTimer = 31;
                }

                // else if (doTransfer && wrist.getPosition() == transfer) {
                //     intakeValue = 1;
                // }
                // if (doIntake || gamepad2.dpad_left || gamepad1.touchpad) {
                //     intakeValue = 1;
                // } else if (gamepad2.dpad_down || gamepad1.left_bumper) {
                //     intakeValue = -1;
                // } else {
                //     intakeValue = 0;
                // }

                leftIntake.setPower(intakeValue);
                rightIntake.setPower(intakeValue);
        }
        public void specimen() {
                // // // Trigger Outtake Slide:
                // // // outtakeSlide.setTargetPosition(slideEnc);
                // // // outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                if (gamepad2.right_trigger > 0 && gamepad2.right_trigger > gamepad2.left_trigger) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        outtakeSlide.setPower(-gamepad2.right_trigger);
                        outtakeSlide2.setPower(-gamepad2.right_trigger);
                }
                else if (gamepad2.left_trigger > 0 && gamepad2.left_trigger > gamepad1.right_trigger) {
                        if (outtakeSlide.getCurrentPosition() < -5 || gamepad1.b) {
                                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                outtakeSlide.setPower(gamepad2.left_trigger);
                                outtakeSlide2.setPower(gamepad2.left_trigger);
                        }
                }
                else if (gamepad1.dpad_right && gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        outtakeSlide.setPower(-gamepad1.right_trigger);
                        outtakeSlide2.setPower(-gamepad1.right_trigger);
                }
                else if (gamepad1.dpad_right && gamepad1.left_trigger > 0 && gamepad1.left_trigger > gamepad1.right_trigger) {
                        if (outtakeSlide.getCurrentPosition() < -5 || gamepad1.b) {
                                outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                outtakeSlide.setPower(gamepad1.left_trigger);
                                outtakeSlide2.setPower(gamepad1.left_trigger);
                        }
                }
                else if (gamepad1.right_stick_button) {
                        if (gamepad1.right_stick_y >= .5 || gamepad1.right_stick_y <= -.5) {
                                if (outtakeSlide.getCurrentPosition() < -5 || gamepad1.b) {
                                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                        outtakeSlide.setPower(gamepad1.right_stick_y);
                                        outtakeSlide2.setPower(gamepad1.right_stick_y);
                                }
                        }
                }
                else if (outtakeSlide.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        // if (outtakeSlide.getCurrentPosition() < -10) {
                        //     outtakeSlide.setPower(-1);
                        //     // outtakeSlide2.setPower(-1);
                        // }
                        outtakeSlide.setPower(-1);
                }
                else {
                        outtakeSlide.setPower(0);
                        outtakeSlide2.setPower(0);
                }

                if (outtakeSlide.getTargetPosition() - outtakeSlide.getCurrentPosition() <= 15 && outtakeSlide.getCurrentPosition() - outtakeSlide.getTargetPosition() <= 15) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (gamepad1.y || gamepad2.y/* || gamepad1.right_bumper*/) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-2000);
                        outtakeSlide2.setPower(-1);
                        // outtakeFlipper.setPosition(0);
                }
                if (gamepad1.b || gamepad2.b) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-500);
                        outtakeSlide2.setPower(-1);
                }
                if (gamepad1.a || gamepad1.x || gamepad2.square || gamepad2.a/* || gamepad1.left_bumper*/) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-200);
                        outtakeSlide2.setPower(1);
                        outtakeFlipper.setPosition(.775);
                }
                if (gamepad1.dpad_down) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        outtakeSlide.setPower(0);
                        outtakeSlide2.setPower(0);
                }



                if (outtakeSlide.getTargetPosition() == outtakeSlide.getCurrentPosition()) {
                        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }


                if (gamepad1.x) {
                        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slideEnc = 0;
                }

                telemetry.addData("outtakeSlide", outtakeSlide.getCurrentPosition());
                telemetry.addData("outtakeSlide Position", slideEnc);

                // // // Button Intake Slide:
                // // // if (gamepad2.circle) extensionPos = Range.clip(extensionPos - 0.005, 0.7, 1);
                // // // else if (gamepad2.square) extensionPos = Range.clip(extensionPos + 0.005, 0.7, 1);
                // if (gamepad2.a) {
                //     extensionPos = .91;
                // }
                // if (gamepad2.b) {
                //     extensionPos = .85;
                // }
                // if (gamepad2.y) {
                //     extensionPos = .71;
                // }
                // if (gamepad2.right_trigger > 0 && gamepad2.right_trigger > gamepad2.left_trigger && extensionPos > .71) {
                //     extensionPos -= gamepad2.right_trigger * .005;
                // }
                // else if (gamepad2.left_trigger > 0 && extensionPos < .91) {
                //     extensionPos += gamepad2.left_trigger * .005;
                // }
                if (!gamepad1.dpad_right && gamepad1.right_trigger > 0 && gamepad1.right_trigger > gamepad1.left_trigger && extensionPos > .71) {
                        extensionPos -= gamepad1.right_trigger * .005;
                }
                else if (!gamepad1.dpad_right && gamepad1.left_trigger > 0 && extensionPos < .91) {
                        extensionPos += gamepad1.left_trigger * .005;
                }

                intakeExtension.setPosition(extensionPos);

                telemetry.addData("extensionPos", extensionPos);
                telemetry.addData("intake extension position", intakeExtension.getPosition());

                // // // Toggle Wrist:
                // // // if (gamepad2.left_bumper) {
                // // //     if (!wristButtonHeld) {
                // // //         wrist.setPosition(wristUp ? .3 : .56);
                // // //         wristUp = !wristUp;
                // // //         wristButtonHeld = true;
                // // //     }
                // // // }
                if (gamepad2.left_bumper) {
                        wrist.setPosition(transfer);
                        // couldHaveSample = false;
                }
                else if (gamepad2.right_bumper) {
                        wrist.setPosition(intakePos);
                        // couldHaveSample = true;

                }
                // else if (wristButtonHeld) {
                //     wristButtonHeld = false;
                // }
                if (gamepad2.right_stick_y != 0) {
                        wrist.setPosition(wrist.getPosition() + gamepad2.right_stick_y * .01);
                        // couldHaveSample = false;
                }
                else if (!gamepad1.right_stick_button) {
                        if (gamepad1.right_stick_y != 0) {
                                wrist.setPosition(wrist.getPosition() + gamepad1.right_stick_y * .01);
                                // couldHaveSample = false;
                        }
                }



                // // // if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                // // //     wrist.setPosition(Range.clip(wrist.getPosition() + gamepad2.left_stick_x, 0, 1));
                // // // }

                // // // // Toggle Flipper:
                if (gamepad1.dpad_left) {
                        outtakeFlipper.setPosition(0.775); // changed from .85 to .95
                }
                if (gamepad1.dpad_up) {
                        outtakeFlipper.setPosition(0);
                }


                if (gamepad1.right_bumper) {
                        rightClaw.setPosition(.65);
                        leftClaw.setPosition(.2);
                }
                if (gamepad1.left_bumper) {
                        rightClaw.setPosition(.3);
                        leftClaw.setPosition(.53);
                }

                if (gamepad2.dpad_up) {
                        doIntake = true;
                }
                if (gamepad1.options) {
                        doIntake = true;
                        wristTimer = 0;
                        wrist.setPosition(intakePos);
                        // couldHaveSample = true;
                }
                wristTimer++;
                if (wristTimer == 15) {
                        couldHaveSample = true;
                }

                if (gamepad2.dpad_down || gamepad1.dpad_down) {
                        intakeValue = -1;
                        doIntake = false;
                }else if (doIntake || doTransfer) {
                        intakeValue = .5;
                }else{
                        intakeValue = 0;
                }
                // if (doIntake && wrist.getPosition() == intakePos) {
                //     couldHaveSample = true;
                // }
                // else if (wrist.getPosition() - intakePos < -.1 || wrist.getPosition() - intakePos > .1) {
                //     couldHaveSample = false;
                // }
                if (colorSensor.alpha() >= 125 && couldHaveSample) {
                        doIntake = false;
                        holdSample = true;
                        couldHaveSample = false;
                }
                if (holdSample) {
                        wrist.setPosition(transfer);
                        extensionPos = .91;
                        readyToTransfer = true;
                        timer++;
                }
                if (holdSample && readyToTransfer && timer > 50) {
                        doTransfer = true;
                }
                if (doTransfer && colorSensor.alpha() < 105) {
                        doTransfer = false;
                        holdSample = false;
                        boxSample = true;
                        wrist.setPosition(.5);
                }
                if (boxSample) {
                        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeSlide.setTargetPosition(-135);
                        boxSample = false;
                }


                if (gamepad1.ps) {
                        slideHome = false;
                        doIntake = false;
                        couldHaveSample = false;
                        holdSample = false;
                        readyToTransfer = false;
                        doTransfer = false;
                        boxSample = false;
                        sampleScored = false;
                        wristTimer = 31;
                }

                // else if (doTransfer && wrist.getPosition() == transfer) {
                //     intakeValue = 1;
                // }
                // if (doIntake || gamepad2.dpad_left || gamepad1.touchpad) {
                //     intakeValue = 1;
                // } else if (gamepad2.dpad_down || gamepad1.left_bumper) {
                //     intakeValue = -1;
                // } else {
                //     intakeValue = 0;
                // }

                leftIntake.setPower(intakeValue);
                rightIntake.setPower(intakeValue);
        }
}
