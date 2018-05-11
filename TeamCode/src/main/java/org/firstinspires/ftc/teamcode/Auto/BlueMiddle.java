package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsHW;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsSensors;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.Locale;

/**
 * Created by Brady on 1/3/2018.
 */

@Autonomous(name =
        "BlueMiddle")


public class BlueMiddle extends OpMode {

    AtomicGearsHW robot = new AtomicGearsHW();
    AtomicGearsSensors sensors = new AtomicGearsSensors();
    Vision vision = new Vision();

    enum State {
        Start, DetectPicture , DriveOff , drive, square, Turn , AlignToCryptobox , PlaceFirstGylph ,
        PLaceGylph, push,JewelArm, KnockOffJewel,BackOff, Straighten, Kill
    }

    State state;
    ElapsedTime time;

    boolean left = false;
    boolean right = false;
    boolean center = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        vision.initVision(hardwareMap);
        state = State.Start;
        time = new ElapsedTime();

    }

    @Override
    public void loop() {
        sensors.ReadColor();
        double CurrentTime = time.time();
        telemetry.addData("time",CurrentTime);
        double gyroangle;
        sensors.angles   = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit,sensors.angles.firstAngle));
        telemetry.addData("Heading",formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        vision.Vision();
        robot.pot.getVoltage();

        final double MeasuredVoltage = robot.pot.getVoltage();
        final double VoltageScale = MeasuredVoltage * 90 / 1.65 ;
        final double Degrees = VoltageScale;

        switch (state) {

            case Start:
                robot.Kill();
                if (robot.IsBusy()) {
                    state = State.JewelArm;
                    time.reset();
                }
                break;
            case JewelArm:
                robot.JewelTilt.setPosition(.3);
                if (robot.JewelTilt.getPosition() >= .3) {
                    robot.JewelPan.setPosition(.5);
                }

                if (robot.JewelPan.getPosition() >= .5 && CurrentTime >= .5) {
                    robot.JewelTilt.setPosition(.88);
                    robot.Kill();
                    time.reset();
                    state = State.KnockOffJewel;
                }
                break;

            case KnockOffJewel:

                if (sensors.ColorIsBlue() && CurrentTime >= 1) {
                    robot.JewelPan.setPosition(.3);
                    state = State.DetectPicture;
                    robot.Kill();
                    time.reset();
                }

                if (sensors.ColorIsRed()&& CurrentTime >= 1) {
                    robot.JewelPan.setPosition(.7);
                    state = State.DetectPicture;
                    robot.Kill();
                    time.reset();
                }
                break;

            case DetectPicture:
                robot.JewelTilt.setPosition(.2);
                robot.JewelPan.setPosition(.5);

                if (vision.Left()) {
                    left = true;
                    robot.Kill();
                    time.reset();
                    state = State.drive;
                    vision.StopVision();
                }
                if (vision.Center()) {
                    center = true;
                    robot.Kill();
                    time.reset();
                    state = State.drive;
                    vision.StopVision();
                }
                if (vision.Right()) {
                    right = true;
                    robot.Kill();
                    time.reset();
                    state = State.drive;
                    vision.StopVision();
                }
                break;
            case drive:
                robot.Reverse(1,24);
                if (robot.DriveDone(24)){ robot.Kill();
                    state = State.square;
                    time.reset();}
                break;

            case square:
                robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.FrontLeft.setPower(-.2);
                robot.FrontRight.setPower(.2);
                robot.BackLeft.setPower(-.2);
                robot.BackRight.setPower(.2);
                if (CurrentTime >= 1){
                    robot.Kill();
                    time.reset();
                    state = State.Straighten;
                }
                break;

            case Straighten:
                robot.TurnAbsoulte(0,gyroangle);
                if ((gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.25)){
                    robot.Kill();
                    state = State.DriveOff;
                    time.reset();
                }
                break;
            case DriveOff:
                if (left)
                {robot.Reverse(1,7.75);
                    if (robot.DriveDone(7.75)){
                        robot.Kill();
                        state = State.Turn;
                        time.reset();
                    }
                }
                if (center)
                {robot.Reverse(1,14.75);
                    if (robot.DriveDone(14.75)){
                        robot.Kill();
                        state = State.Turn;
                        time.reset();
                    }
                }
                if (right)
                {robot.Reverse(1,22.25);
                    if (robot.DriveDone(22.25)){
                        robot.Kill();
                        state = State.Turn;
                        time.reset();
                    }
                }

                break;

            case Turn:
                robot.TurnAbsoulte(-90,gyroangle);
                if ((gyroangle <= -88 && gyroangle >= -92 && CurrentTime >= 1.25)){
                    robot.Kill();
                    state = State.AlignToCryptobox;
                    time.reset();
                }
                break;

            case AlignToCryptobox:
                robot.DriveWithPower(.4);
                if (CurrentTime >= 1){
                    robot.LeftBeltIntakeServo.setPower(.5);
                    robot.RightBeltIntakeSevo.setPower(-.5);
                    robot.GylphIntakeLeft.setPower(1);
                    robot.GylphIntakeRight.setPower(-1);
                    robot.Kill();
                    state = State.PlaceFirstGylph;
                    time.reset();
                }
                break;

            case PlaceFirstGylph:
                robot.Reverse(.5,13);
                robot.LeftBeltIntakeServo.setPower(.5);
                robot.RightBeltIntakeSevo.setPower(-.5);
                robot.GylphIntakeLeft.setPower(1);
                robot.GylphIntakeRight.setPower(-1);
                if (robot.DriveDone(13)){
                    robot.LeftBeltIntakeServo.setPower(0);
                    robot.RightBeltIntakeSevo.setPower(0);
                    robot.GylphIntakeLeft.setPower(0);
                    robot.GylphIntakeRight.setPower(0);
                    robot.Kill();
                    state = State.PLaceGylph;
                    time.reset();
                }
                break;

            case PLaceGylph:
                robot.TurnAbsoulte(0,gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.25){
                    robot.Kill();
                    state = State.push;
                    time.reset();
                }
                break;

            case push:
                robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.FrontLeft.setPower(-.6);
                robot.FrontRight.setPower(-.6);
                robot.BackLeft.setPower(.6);
                robot.BackRight.setPower(.6);
                if (Degrees <= 4){robot.Flippy.setPower(0);}
                else {robot.Flippy.setPower(.5);}
                if (CurrentTime >= 1){
                    robot.Kill();
                    state = State.BackOff;
                    time.reset();
                }
                break;

            case BackOff:
                robot.Left(1,4);
                if (robot.StrafeDone(4)){
                    robot.Kill();
                    state = State.Kill;
                    time.reset();
                }
                break;

            case Kill: robot.Kill();
                time.reset();
                break;
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

