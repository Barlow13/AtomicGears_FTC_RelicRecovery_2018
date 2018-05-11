/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestCode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;


/**
 *
 * This particular OpMode just executes a basic Holonomic Drive Teleop for a Four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 */

@TeleOp(name="Linear Drive", group="Linear Opmode")
@Disabled
public class TestLinear extends LinearOpMode {

    private DcMotor motorFrontRight = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorBackLeft = null;
    private Servo Left = null;
    private Servo Right = null;

    @Override
    public void runOpMode() {

        /**
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        motorFrontLeft = hardwareMap.get(DcMotor.class, "fl");
        motorFrontRight = hardwareMap.get(DcMotor.class, "fr");
        motorBackLeft = hardwareMap.get(DcMotor.class, "bl");
        motorBackRight = hardwareMap.get(DcMotor.class, "br");
        Left = hardwareMap.get(Servo.class,"ls");
        Right = hardwareMap.get(Servo.class,"rs");


        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper){Left.setPosition(1);
            Right.setPosition(0);}
            else if (gamepad1.left_bumper){Left.setPosition(0);
            Right.setPosition(1);}

            double FrontLeft;
            double FrontRight;
            double BackRight;
            double BackLeft;

            double gamepad1LeftY = -gamepad1.left_stick_y;
            double gamepad1LeftX = gamepad1.left_stick_x;
            double gamepad1RightX = gamepad1.right_stick_x;

            // holonomic formulas
            // clip the right/left values so that the values never exceed +/- 1

            FrontRight = Range.clip(gamepad1LeftY - gamepad1LeftX - gamepad1RightX, -1, 1);
            FrontLeft = Range.clip(-gamepad1LeftY - gamepad1LeftX - gamepad1RightX, -1, 1);
            BackLeft = Range.clip(-gamepad1LeftY + gamepad1LeftX - gamepad1RightX, -1, 1);
            BackRight = Range.clip(gamepad1LeftY + gamepad1LeftX - gamepad1RightX, -1, 1);

            FrontLeft = (float) scaleInput(FrontLeft);
            FrontRight = (float) scaleInput(FrontRight);
            BackLeft = (float) scaleInput(BackLeft);
            BackRight =  (float) scaleInput(BackRight);
            // write the values to the motors

            motorFrontRight.setPower(FrontRight);
            motorFrontLeft.setPower(FrontLeft);
            motorBackLeft.setPower(BackLeft);
            motorBackRight.setPower(BackRight);

            /**
             * Telemetry for debugging
             */

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " +
                    String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
            telemetry.update();


        }
    }

    /*
 * This method scales the joystick input so for low joystick values, the
 * scaled value is less than linear.  This is to make it easier to drive
 * the robot more precisely at slower speeds.
 */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}