package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * Created by Brady on 1/3/2018.
 */

@TeleOp(name = "calibrate")


public class calibrate extends OpMode {

    AtomicGearsHW robot = new AtomicGearsHW();
    AtomicGearsSensors sensors = new AtomicGearsSensors();
    Vision vision = new Vision();
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cRangeSensor rangeSensor2;
    ModernRoboticsI2cRangeSensor wall;
    ElapsedTime time;

    boolean left = false;
    boolean right = false;
    boolean center = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        vision.initVision(hardwareMap);
        rangeSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r2");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        wall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"wrs");

        time = new ElapsedTime();

    }

    @Override
    public void loop() {
        sensors.ReadColor();
        double CurrentTime = time.time();
        telemetry.addData("time",CurrentTime);
        sensors.angles   = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading",formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        vision.Vision();
        robot.pot.getVoltage();

        final double MeasuredVoltage = robot.pot.getVoltage();
        final double VoltageScale = MeasuredVoltage * 90 / 1.65 ;
        final double Degrees = VoltageScale;

        telemetry.addData("wall",wall.getDistance(DistanceUnit.CM));
        telemetry.addData("CM",rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("CM2",rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.addData("Left","Left =" + vision.Left());
        telemetry.addData("Center","Center =" + vision.Center());
        telemetry.addData("Right","Right =" + vision.Right());
        telemetry.addData("degrees",Degrees);
        telemetry.addData("volt",robot.pot.getVoltage());
        telemetry.addData("hue" , sensors.hsvValues[0]);
        telemetry.update();


    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

