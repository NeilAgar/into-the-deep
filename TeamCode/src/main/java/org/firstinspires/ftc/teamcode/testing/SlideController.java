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

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Reset Slide TeleOp")
public class SlideController extends LinearOpMode {
    DcMotor outtakeSlide;
    DcMotor outtakeSlide2;

    @Override
    public void runOpMode() throws InterruptedException {
        if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            outtakeSlide.setPower(1); outtakeSlide2.setPower(1);
        } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
            outtakeSlide.setPower(-1); outtakeSlide2.setPower(-1);
        }
    }
}
