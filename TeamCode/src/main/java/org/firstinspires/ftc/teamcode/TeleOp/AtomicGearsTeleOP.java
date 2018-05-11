package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsHW;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsSensors;

import java.util.Locale;

@TeleOp(name="AtomicGears")


public class AtomicGearsTeleOP extends LinearOpMode {

    AtomicGearsHW robot = new AtomicGearsHW();

    boolean claw = false;
    boolean toggle = true;
    boolean lift = false;
    boolean toggle2 = true;

    @Override

    public void runOpMode() {

        robot.init(hardwareMap);



        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {



            if (toggle2 && gamepad1.y) {  // Only execute once per Button push
                toggle2 = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (lift) {
                    lift= false;
                } else {
                    lift = true;
                }
            } else if(gamepad1.y == false) {
                toggle2 = true; // Button has been released, so this allows a re-press to activate the code above.
            }

            if (lift && !robot.top.getState() == false){
                robot.Gylpht.setPower(-1);
            }
            else if (!lift && !robot.bottom.getState() == false){
                robot.Gylpht.setPower(1);
            }
            else {
                robot.Gylpht.setPower(0);
            }

            if (gamepad1.b){
                robot.Flippy.setPower(.5);
            }
            else if(gamepad1.x){
                robot.Flippy.setPower(-.5);
            }
            else {robot.Flippy.setPower(0);}

            if (RightTrigger()){
                robot.LeftBeltIntakeServo.setPower(.5);
                robot.RightBeltIntakeSevo.setPower(-.5);
                robot.GylphIntakeLeft.setPower(1);
                robot.GylphIntakeRight.setPower(-1);
            }

            else if (LeftTrigger()){
                robot.LeftBeltIntakeServo.setPower(-.5);
                robot.RightBeltIntakeSevo.setPower(.5);
                robot.GylphIntakeLeft.setPower(-1);
                robot.GylphIntakeRight.setPower(1);
                }


            else if (gamepad1.left_bumper){
                robot.GylphIntakeLeft.setPower(1);
                robot.GylphIntakeRight.setPower(1);
                robot.LeftBeltIntakeServo.setPower(.5);
                robot.RightBeltIntakeSevo.setPower(.5);
            }

            else if (gamepad1.right_bumper){
                robot.GylphIntakeLeft.setPower(-1);
                robot.GylphIntakeRight.setPower(-1);
                robot.LeftBeltIntakeServo.setPower(-.5);
                robot.RightBeltIntakeSevo.setPower(-.5);
            }

            else {
                robot.GylphIntakeLeft.setPower(0);
                robot.GylphIntakeRight.setPower(0);
                robot.LeftBeltIntakeServo.setPower(0);
                robot.RightBeltIntakeSevo.setPower(0);}

                if (gamepad1.start){
                robot.JewelTilt.setPosition(.85);
                robot.JewelPan.setPosition(.5);
                }
                else {robot.JewelTilt.setPosition(.2);
                    robot.JewelPan.setPosition(.5);}

            double gamepad1LeftY = -gamepad1.left_stick_y;
            double gamepad1LeftX = gamepad1.left_stick_x;
            double gamepad1RightX = gamepad1.right_stick_x;
            double FrontLeft;
            double FrontRight;
            double BackRight;
            double BackLeft;

            FrontRight = Range.clip(gamepad1LeftY - gamepad1LeftX - gamepad1RightX, -1, 1);
            FrontLeft = Range.clip(-gamepad1LeftY - gamepad1LeftX - gamepad1RightX, -1, 1);
            BackLeft = Range.clip(-gamepad1LeftY + gamepad1LeftX - gamepad1RightX, -1, 1);
            BackRight = Range.clip(gamepad1LeftY + gamepad1LeftX - gamepad1RightX, -1, 1);

            FrontLeft = (float) robot.scaleInput(FrontLeft);
            FrontRight = (float) robot.scaleInput(FrontRight);
            BackLeft = (float) robot.scaleInput(BackLeft);
            BackRight =  (float) robot.scaleInput(BackRight);

            robot.FrontLeft.setPower(FrontLeft);
            robot.FrontRight.setPower(FrontRight);
            robot.BackLeft.setPower(BackLeft);
            robot.BackRight.setPower(BackRight);

            if (gamepad2.x){
                robot.Shoulder.setPosition(.28);
                robot.Base.setPosition(0);
            }

            if (gamepad2.y){
                robot.Shoulder.setPosition(.28);
                robot.Base.setPosition(.19);
            }
            if (gamepad2.b){
                robot.Base.setPosition(0);
                robot.Shoulder.setPosition(0);
            }


            double elbow = -gamepad2.left_stick_y;
            double wrist = -gamepad2.right_stick_y;

            elbow = Range.clip(elbow,-.5,.5);
            wrist = Range.clip(wrist,-.5,.5);

            robot.Elbow.setPower(elbow);
            robot.Wrist.setPower(wrist);

            if (gamepad2.right_bumper){
                robot.ClawRotation.setPosition(.5);}
            else if (gamepad2.left_bumper){
                robot.ClawRotation.setPosition(1);}

            if (toggle && gamepad2.a) {  // Only execute once per Button push
                toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (claw) {
                    claw= false;
                    robot.Claw.setPosition(1);
                } else {
                    claw = true;
                    robot.Claw.setPosition(0);
                }
            } else if(gamepad2.a == false) {
                toggle = true; // Button has been released, so this allows a re-press to activate the code above.
            }

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

    boolean RightTrigger(){
        if (gamepad1.right_trigger >= 0.1){
            return (true);}
        else return (false);}

    boolean LeftTrigger(){
        if (gamepad1.left_trigger >= 0.1){
            return (true);}
        else return (false);}
}