package org.firstinspires.ftc.teamcode.SubSystems;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

public class Vision {

    public VuforiaLocalizer vuforia;

    public VuforiaTrackables relicTrackables;

    public VuforiaTrackable relicTemplate;

    public RelicRecoveryVuMark vuMark;

    HardwareMap hwMap = null;

    public Vision() {
    }

    public void initVision(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters1.vuforiaLicenseKey = "AeGzv1L/////AAAAGVITNWG9l0vpn4p4pvlP/jcpnVwgp6LntDGpzlU29IDkrI1lgYfUDb1aMdcWejc2SPKfy8OAGx1jdELMNfkwxKrIyrAt0Udc6N5gERLf+5804xm3ZRG5yh7nY+ttX0UL8FZLEajtXW2XGLzlFZEuB2yHrvjFYFheO1tCl6wuGyQZdVSaMuJyeIYeyvTYAJNA38/ZsH8oYjKl/VIIV7KTx8CNAcevdBu6VIrtHTXEXbTVIb5IahynsMvY1vESupkdoPAqWwbmdnnRwaCOSD1HccaqxXnFGi595AZ5MhJb2CiZoiDjV5HnayBOXfeiK/PU2nyrCANHM1VkN6zKhA0i8rGKNwclST6uqAfosEmU62Xf";
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters1);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    public void Vision() {
        relicTrackables.activate();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
    }

    public void StopVision() {
        relicTrackables.deactivate();}

    public boolean Left() {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return (true);
        } else return (false);
    }

    public boolean Center() {
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            return (true);
        } else return (false);
    }

    public boolean Right() {
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return (true);
        } else return (false);
    }
}


