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
    //private List<Recognition> finalRecognitions = new ArrayList<>();
    private List<Recognition> recognitions;

    private LinearOpMode caller;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    //    private static final int LOWER_X_BOUNDARY = 0; //temporary values
//    private static final int UPPER_X_BOUNDARY = 300;
//    private static final int LOWER_Y_BOUNDARY = 250;
//    private static final int UPPER_Y_BOUNDARY = 390;
    private static final int MID_BOUND = 200;
    private static final int LEFT_BOUND = 0, TOP_BOUND = 0, RIGHT_BOUND = 200, BOTTOM_BOUND = 0;

    private SkyStoneLocation location;

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public Sensors(LinearOpMode caller) {
        this.caller = caller;
        telemetry = caller.telemetry;
        hardwareMap = caller.hardwareMap;
    }

    @Override
    public void init() {

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "C310"); //webcam name "C310"

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE//,
                //LABEL_SECOND_ELEMENT
        );

        tfod.setClippingMargins(LEFT_BOUND, TOP_BOUND, RIGHT_BOUND, BOTTOM_BOUND);

        tfod.activate();
        telemetry.addData("Subassembly: ", "Sensors initialized!");

    }

    @Override
    public void status() {

    }

    public SkyStoneLocation findSkystone() {

        if (tfod != null) {

            //Poll recognitions 5 times with break in between for lighting.
            //Each recognition is considered as a stone.
            for (int i = 0; i < 5; i++) {
                recognitions = tfod.getRecognitions();
                caller.sleep(100);
            }

            telemetry.addData("Recognitions: ", recognitions.toString());
            telemetry.update();


            if (recognitions.size() == 0) {  //zero stones in sight-- this should never happen
                location = SkyStoneLocation.LEFT; //default case
            }

            //If there is more than one stone recognized, the skystone must be on the right.
            else if (recognitions.size() > 1) {  //two stones in sight s s k
                location = Sensors.SkyStoneLocation.RIGHT;
            }

            else if ((recognitions.get(0).getLeft() + recognitions.get(0).getRight()) / 2 < MID_BOUND) {
                location = Sensors.SkyStoneLocation.CENTER;  // s k s
            }

            else {
                location = Sensors.SkyStoneLocation.LEFT;
            }
        }

        recognitions.clear();

        telemetry.addData("SkyStone Location: ", location);
        telemetry.update();

        return location;
    }


    public void shutdown() {
        tfod.shutdown();
    }

    public enum SkyStoneLocation {

        LEFT,

        CENTER,

        RIGHT

    }
}
