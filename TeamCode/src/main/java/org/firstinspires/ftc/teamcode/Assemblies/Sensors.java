package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABEL_STONE;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Assemblies.ConfigurationData.VUFORIA_KEY;

public class Sensors implements Subassembly {
    private List<Recognition> recognitions;
    private LinearOpMode caller;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static final int LOWER_X_BOUNDARY = 0;
    private static final int UPPER_X_BOUNDARY = 200;
    private static final int LOWER_Y_BOUNDARY = 0;
    private static final int UPPER_Y_BOUNDARY = 0;
    private static final int MIDDLE_BOUNDARY = 200;

    private SkyStoneLocation location;

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    //constructor
    public Sensors(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
        hardwareMap = caller.hardwareMap;
    }

    @Override
    public void init() {

        //  Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        hardwareMap = caller.hardwareMap;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //  Link webcam, name C310
        parameters.cameraName = caller.hardwareMap.get(WebcamName.class, "C310");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //decrease conf 0.8
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);


        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE//,
                //LABEL_SECOND_ELEMENT
        );

        // Clips window so the detector can only "see" the first two stones
        tfod.setClippingMargins(LOWER_X_BOUNDARY, UPPER_X_BOUNDARY, LOWER_Y_BOUNDARY, UPPER_Y_BOUNDARY);

        tfod.activate();

        telemetry.addData("Subassembly:", "Sensors initialized!");
        telemetry.update();

    }

    @Override
    public void status() {

    }

    /**
     * Examines first two stones using TensorFlowObjectDetector
     * and uses logic to find the skystone among the first three.
     * @return Skystone location enum type
     */
    public SkyStoneLocation findSkystone() {

        if (tfod != null) {
            //  Poll recognitions 5 times with break in between for lighting
            //TODO blocking out the pixels
            for (int i = 0; i < 5; i++) {
                recognitions = tfod.getRecognitions();
                caller.sleep(100);
            }

            telemetry.addData("Recognitions: ", recognitions.toString());
            telemetry.update();

            //  If there is more than one stone recognized, the skystone must be on the right.
            if (recognitions.size() > 1) {
                location = SkyStoneLocation.RIGHT;
            } else if ((recognitions.get(0).getLeft() + recognitions.get(0).getRight()) / 2 < MIDDLE_BOUNDARY) {
                //  If the the center of the stone is less than a certain middle boundary,
                //  then the skystone must be the right of the two stones: the center one overall.
                location = SkyStoneLocation.CENTER;
            } else {  //  If the above two cases are not true, then the skystone must be on the very left.
                location = SkyStoneLocation.LEFT;
            }

            //TODO swap with return

            //  Displays location on phone
            switch (location) {
                case LEFT:
                    telemetry.addData("SkyStone Location:", "LEFT");
                    break;
                case RIGHT:
                    telemetry.addData("SkyStone Location:", "RIGHT");
                    break;
                case CENTER:
                    telemetry.addData("SkyStone Location:", "CENTER");
                    break;
            }
        }

        telemetry.update();

        return location;
    }

    public void shutdown() {
        tfod.shutdown();
    }

    /**
     * The possible locations for a skystone.
     */
    public enum SkyStoneLocation {

        LEFT,

        CENTER,

        RIGHT
    }
}
