package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsHW;
import org.firstinspires.ftc.teamcode.SubSystems.AtomicGearsSensors;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.Locale;

/**
 * Created by Brady on 3/28/2018.
 */

@Autonomous(name =
        "RedCornerMG")
public class RedCornerMulti extends OpMode {

    AtomicGearsHW robot = new AtomicGearsHW();
    AtomicGearsSensors sensors = new AtomicGearsSensors();
    Vision vision = new Vision();

    enum State {
        Start, DetectPicture , DriveOff , drive, square, Turn , AlignToCryptobox , PlaceFirstGylph , PLaceGylph, push, GetMoreGylphs, ComeBack, HitBox, PlaceMoreGylphs, Jitter, Jitter2, JewelArm, KnockOffJewel, SquareUpAgain, DriveForwardMore, BackOff, Straighten, RamPile, hitbox, placegylph, Recenter, DriveBack, GetGylphs, align, StrafeToPad, CenterPit, EnterPit, fixgylph, CenterBox, ExitPit, Kill
    }

    State state;
    ElapsedTime time;
    ModernRoboticsI2cRangeSensor range;
    AnalogInput a;
    ModernRoboticsI2cRangeSensor wall;

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
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"r2");
        a = hardwareMap.analogInput.get("a");
        wall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"wrs");
    }

    @Override
    public void loop() {
        sensors.ReadColor();
        double CurrentTime = time.time();
        double gyroangle;
        sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        vision.Vision();
        robot.pot.getVoltage();
        double cm = range.getDistance(DistanceUnit.CM);
        double cm2 = wall.getDistance(DistanceUnit.CM);
        double GylphDetctor = a.getVoltage();

        final double MeasuredVoltage = robot.pot.getVoltage();
        final double VoltageScale = MeasuredVoltage * 90 / 1.65;
        final double Degrees = VoltageScale;

        switch (state) {

            case Start:
                robot.Kill();
                if (Degrees <= 4){robot.Flippy.setPower(0);
                    robot.LeftBeltIntakeServo.setPower(-.5);
                    robot.RightBeltIntakeSevo.setPower(.5);
                    robot.GylphIntakeLeft.setPower(-1);
                    robot.GylphIntakeRight.setPower(1);
                    state = State.JewelArm;
                    time.reset();}
                else {robot.Flippy.setPower(.5);}
                break;
            case JewelArm:
                robot.JewelTilt.setPosition(.3);
                if (robot.JewelTilt.getPosition() >= .3) {
                    robot.JewelPan.setPosition(.5);
                }

                if (robot.JewelPan.getPosition() >= .5 && CurrentTime >= .5) {
                    robot.JewelTilt.setPosition(.87);
                    robot.Kill();
                    time.reset();
                    state = State.KnockOffJewel;
                }
                break;

            case KnockOffJewel:

                if (sensors.ColorIsBlue() && CurrentTime >= 1) {
                    robot.JewelPan.setPosition(.7);
                    state = State.DetectPicture;
                    robot.Kill();
                    time.reset();
                }

                if (sensors.ColorIsRed() && CurrentTime >= 1) {
                    robot.JewelPan.setPosition(.3);
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
                robot.Left(.4,40);
                if (robot.StrafeDone(40)){
                    robot.Kill();
                    time.reset();
                    state = State.CenterPit;
                }
                break;

            case CenterPit:
                robot.TurnAbsoulte(-20,gyroangle);
                if (gyroangle <= -18 && gyroangle >= -22 && CurrentTime >= 1.25){
                    robot.Kill();
                    state = State.EnterPit;
                    time.reset();
                }
                break;

            case EnterPit:
                robot.DriveWithPower(.4);
                if (CurrentTime >= 2.5 || GylphDetctor >= .2 ){
                    robot.Kill();
                    state = State.ExitPit;
                    time.reset();

                }
                break;

            case ExitPit:
                robot.Reverse(1,35);
                if (robot.DriveDone(35)){
                    robot.Kill();
                    state = State.align;
                    time.reset();

                }
                break;

            case align:
                robot.TurnAbsoulte(0,gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.25){
                    robot.Kill();
                    state = State.AlignToCryptobox;
                    time.reset();
                }
                break;

            case AlignToCryptobox:
                robot.Wall(30,cm2,2);
                if(cm2>= 29 && cm2<= 31){
                    robot.LeftBeltIntakeServo.setPower(0);
                    robot.RightBeltIntakeSevo.setPower(0);
                    robot.GylphIntakeLeft.setPower(0);
                    robot.GylphIntakeRight.setPower(0);
                    robot.Kill();
                    state = State.StrafeToPad;
                    time.reset();
                }
                if (cm2<=5){
                    robot.Kill();
                    state = State.DriveBack;
                    time.reset();
                }
                break;

            case DriveBack:
                robot.Reverse(.5,30);
                if (robot.DriveDone(30)){
                    robot.LeftBeltIntakeServo.setPower(0);
                    robot.RightBeltIntakeSevo.setPower(0);
                    robot.GylphIntakeLeft.setPower(0);
                    robot.GylphIntakeRight.setPower(0);
                    robot.Kill();
                    state = State.StrafeToPad;
                    time.reset();
                }
                break;

            case StrafeToPad:
                robot.Right(.4,24);
                if (robot.StrafeDone(24)){
                    robot.Kill();
                    time.reset();
                    state = State.CenterBox;
                }
                break;


            case CenterBox:

                if (right){
                    robot.Wall(48,cm ,0);
                    if (cm >= 47 && cm <= 49){
                        robot.Kill();
                        state = State.Recenter;
                        time.reset();
                    }
                }
                if (center){
                    robot.Wall(66,cm,0);
                    if (cm >= 65 && cm <= 67){
                        robot.Kill();
                        state = State.Recenter;
                        time.reset();
                    }
                }
                if (left){
                    robot.Wall(83,cm,0);
                    if (cm >= 82 && cm <= 84){
                        robot.Kill();
                        state = State.Recenter;
                        time.reset();
                    }
                }
                break;

            case Recenter:
                robot.TurnAbsoulte(0,gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.25){
                    robot.Kill();
                    state = State.placegylph;
                    time.reset();
                }
                break;

            case placegylph:
                if (Degrees >= 33){robot.Flippy.setPower(0);
                    robot.DriveWithPower(-.3);
                }
                else {robot.Flippy.setPower(-.5);}
                if (CurrentTime >= 3){
                    robot.Kill();
                    state = State.BackOff;
                    time.reset();}
                break;

            case BackOff:
                robot.LeftBeltIntakeServo.setPower(0);
                robot.RightBeltIntakeSevo.setPower(0);
                robot.GylphIntakeLeft.setPower(0);
                robot.GylphIntakeRight.setPower(0);
                robot.Forward(1,6);
                if (robot.DriveDone(6)){
                    robot.Kill();
                    state = State.Kill;
                    time.reset();
                }
                break;

            case Kill:
                robot.Kill();
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
