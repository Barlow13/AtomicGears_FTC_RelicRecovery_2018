package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Brady on 4/5/2018.
 **/
@TeleOp(name = "Analog")
@Disabled
public class Analog extends LinearOpMode {

    // this is a custom wrote code showing how to take an analog ultrasonic
    //sensor and using simple math to take a voltage reading and turn it into inches

    /*
     * Main loop
     */

    @Override
    public void runOpMode() throws InterruptedException {

        AnalogInput a;
        a = hardwareMap.analogInput.get("a");


        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            a.getVoltage();

            final double MeasuredVoltage = a.getVoltage();

            telemetry.addData("volt",a.getVoltage());
            telemetry.update();
        }
    }
}

