package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@TeleOp(name="TeleopCvAlg", group="Fx")
//@Disabled
public class TeleopCVAlg extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String VUFORIA_KEY = "ASLOtt3/////AAABmbzx65TV2UrVqNnUS424xZpOs/Vw2BUdY1equY69euPD199BJxppV5RLjqwvUYyCtWjtNqI1CTL6Vlp5RvY5Cimm92p/ML2lDhM0GR/f2feDTFMgXLGPiEs7qStp839UrN8YNxDHbOdQHMCIlJeouhxOh9Y87rubm14L7RwrdvOyfo9v8o5ZyFqH33ap58P9xdmhIitqvU2nmVjieMZoTfGLuu0Fmuls+u3bHv5OfEcj8cEUlJ02sui0qdjfNcJIOPkNZUh822tYespPWEqEOLeOf3wXvy5skSQplg/1JOxPXdq8HUcCqeo25hL8iXkg1tlw12aCTLNkQli80Hw8Jiddnl1oKQe7cBziTEIXKmBW";
    private List<Recognition> recognitions;

    private static final int LOWER_X_BOUNDARY = 0;
    private static final int UPPER_X_BOUNDARY = 1;
    private static final int LOWER_Y_BOUNDARY = 0;
    private static final int UPPER_Y_BOUNDARY = 1;
    private SkystoneLocation location;

    private VuforiaLocalizer vuf;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if (tfod != null) {
                for (int i = 0; i < 5; i++) {
                    recognitions = tfod.getRecognitions();
                    sleep(100);
                }
                //pruning recognitions that are outside of focus area
                for (Recognition recognition : recognitions) {
                    if (recognition.getLeft() > UPPER_X_BOUNDARY ||
                            recognition.getLeft() < LOWER_X_BOUNDARY) {
                        recognitions.remove(recognition);
                    } else if (recognition.getBottom() > UPPER_Y_BOUNDARY ||
                            recognition.getBottom() < LOWER_Y_BOUNDARY) {
                        recognitions.remove(recognition);
                    }
                }

                if (recognitions.size() > 1) {

                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "C310");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //decrease conf o.8
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT//,
                //LABEL_SECOND_ELEMENT
        );
    }
}
