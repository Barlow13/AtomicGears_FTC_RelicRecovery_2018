package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Brady on 3/5/2018.
 */
@TeleOp(name = "pot")
@Disabled
public class pot extends LinearOpMode {

    // this is a custom wrote code showing how to take an analog ultrasonic
    //sensor and using simple math to take a voltage reading and turn it into inches

    /*
     * Main loop
     */

    @Override
    public void runOpMode() throws InterruptedException {

        AnalogInput pot;
        pot = hardwareMap.analogInput.get("p");


        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            pot.getVoltage();

            final double MeasuredVoltage = pot.getVoltage();
            final double VoltageScale = MeasuredVoltage / 0.01368351 ;
            final double Degrees = VoltageScale;


            telemetry.addData("inches",Degrees);
            telemetry.addData("volt",pot.getVoltage());
            telemetry.update();
        }
    }
}

