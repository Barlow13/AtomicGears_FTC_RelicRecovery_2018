package org.firstinspires.ftc.teamcode.SubSystems;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

public class AtomicGearsSensors extends AtomicGearsHW {

    public ColorSensor sensorColor = null;

    public float hsvValues[] = {0F, 0F, 0F};

    public final float values[] = hsvValues;

    public final double SCALE_FACTOR = 255;

    public BNO055IMU imu;

    public Orientation angles;

    HardwareMap hwMap = null;

    public AtomicGearsSensors(){}

    public void initSensors(HardwareMap ahwMap) {
        hwMap = ahwMap;
        sensorColor = hwMap.get(ColorSensor.class, "cs");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void ReadColor() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
    }

    public boolean ColorIsBlue() {
        if (hsvValues[0] >= 190 && hsvValues[0] <= 300 ) {
            return (true);
        } else return (false);
    }

    public boolean ColorIsRed() {
        if (hsvValues[0] >= 340 && hsvValues[0] <= 370 || hsvValues[0] >= 0 && hsvValues[0] <= 40 ) {
            return (true);
        } else return (false);
    }
}



