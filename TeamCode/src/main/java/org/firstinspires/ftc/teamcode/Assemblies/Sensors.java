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
    private List<Recognition> finalRecognitions = new ArrayList<>();
    private List<Recognition> recognitions;
    private LinearOpMode caller;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static final int LOWER_X_BOUNDARY = 0; //temporary values
    private static final int UPPER_X_BOUNDARY = 300;
    private static final int LOWER_Y_BOUNDARY = 250;
    private static final int UPPER_Y_BOUNDARY = 390;
    private static final int MIDDLE_BOUNDARY = 200;
    private int left = 0, top = 0, right = 200, bottom = 0;

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
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        hardwareMap = caller.hardwareMap;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //webcam name must be c310
        parameters.cameraName = caller.hardwareMap.get(WebcamName.class, "C310");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //decrease conf o.8
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        //only detecting stones

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE//,
                //LABEL_SECOND_ELEMENT
        );

        tfod.setClippingMargins(left,top,right,bottom);

        tfod.activate();
        telemetry.addData("Subassembly", "Sensors initialized!");

    }

    @Override
    public void status() {

    }

    public SkyStoneLocation findSkystone() {
        //tfod.activate();
        if (tfod != null) {
            //poll recognitions 5 times with break in between for lighting
            if (true //gamepad1.a
            ) {
                //TODO blocking out the pixels
                //tfod.setClippingMargins(LOWER_X_BOUNDARY, UPPER_X_BOUNDARY, LOWER_Y_BOUNDARY, UPPER_Y_BOUNDARY);
                for (int i = 0; i < 5; i++) {
                    recognitions = tfod.getRecognitions();
                    caller.sleep(100);
                }

                telemetry.addData("recognitions", recognitions.toString());
                telemetry.update();

                //This loop prunes recognitions that are outside of viewing window, which is limited
                //to the area around the first two stones.
                if(recognitions == null || recognitions.size() != 0) {
                    for (int i = 0; i<recognitions.size(); i++) {
                        if ( true
//                                !((recognitions.get(i).getLeft() > UPPER_X_BOUNDARY || recognitions.get(i).getLeft() < LOWER_X_BOUNDARY)
//                                        && (recognitions.get(i).getBottom() > UPPER_Y_BOUNDARY || recognitions.get(i).getBottom() < LOWER_Y_BOUNDARY))
//
                        )
                        {
                            //recognitions.remove(recognition);
                            finalRecognitions.add(recognitions.get(i));
                        }
                    }
                }
                else {
                    //DEFAULT CASE
                    return SkyStoneLocation.LEFT;
                }



                //If there is more than one stone recognized, the skystone must be on the right.
                // s s k
                if (finalRecognitions.size() > 1) {
                    location = Sensors.SkyStoneLocation.RIGHT;
                }
                // s k s
                // this must return center, as if the get left boundary is less than the middle,
                // since we are detecting stones, it is center
                //now we are using the average
                else if ((finalRecognitions.get(0).getLeft() + finalRecognitions.get(0).getRight()) / 2 < MIDDLE_BOUNDARY) {
                    location = Sensors.SkyStoneLocation.CENTER;
                }
                // k s s
                else {
                    location = Sensors.SkyStoneLocation.LEFT;
                }
                //TODO swap with return
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
            finalRecognitions.clear();
            telemetry.update();
        }
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
