package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Created by Brady on 9/10/2017.
 */

@TeleOp(name="Detection", group ="Concept")
@Disabled


public class Detection extends LinearOpMode {

    Vision vision = new Vision();

    @Override public void runOpMode(){

        vision.initVision(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            vision.Vision();

            telemetry.addData("Left","Left =" + vision.Left());
            telemetry.addData("Center","Center =" + vision.Center());
            telemetry.addData("Right","Right =" + vision.Right());
            telemetry.update();

        }
    }

}

