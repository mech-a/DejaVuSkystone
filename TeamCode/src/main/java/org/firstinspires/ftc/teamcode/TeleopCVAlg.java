package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Assemblies.Sensors;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TeleopCvAlg", group="Fx")
//@Disabled
public class TeleopCVAlg extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String VUFORIA_KEY = "ASLOtt3/////AAABmbzx65TV2UrVqNnUS424xZpOs/Vw2BUdY1equY69euPD199BJxppV5RLjqwvUYyCtWjtNqI1CTL6Vlp5RvY5Cimm92p/ML2lDhM0GR/f2feDTFMgXLGPiEs7qStp839UrN8YNxDHbOdQHMCIlJeouhxOh9Y87rubm14L7RwrdvOyfo9v8o5ZyFqH33ap58P9xdmhIitqvU2nmVjieMZoTfGLuu0Fmuls+u3bHv5OfEcj8cEUlJ02sui0qdjfNcJIOPkNZUh822tYespPWEqEOLeOf3wXvy5skSQplg/1JOxPXdq8HUcCqeo25hL8iXkg1tlw12aCTLNkQli80Hw8Jiddnl1oKQe7cBziTEIXKmBW";
    private List<Recognition> finalRecognitions = new ArrayList<>();
    private List<Recognition> recognitions;

    //left 100
    //right 625
    //top 225
    //bottom 385

    private static final int LOWER_X_BOUNDARY = 100; //temporary values
    private static final int UPPER_X_BOUNDARY = 625;
    private static final int LOWER_Y_BOUNDARY = 225;
    private static final int UPPER_Y_BOUNDARY = 385;
    private static final int MIDDLE_BOUNDARY = 220; // 50 and 215

    //


    private Sensors.SkyStoneLocation location;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforiaWC();
        //initVuforia();


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

        while (opModeIsActive()) {
            if (tfod != null) {
                //poll recognitions 5 times with break in between for lighting
                if (true //gamepad1.a
                ) {
                    //TODO blocking out the pixels
                    //tfod.setClippingMargins(LOWER_X_BOUNDARY, UPPER_X_BOUNDARY, LOWER_Y_BOUNDARY, UPPER_Y_BOUNDARY);
                    for (int i = 0; i < 5; i++) {
                        recognitions = tfod.getRecognitions();
                        sleep(100);
                    }

                    telemetry.addData("recognitions", recognitions.toString());
                    telemetry.update();

                    //This loop prunes recognitions that are outside of viewing window, which is limited
                    //to the area around the first two stones.
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
                sleep(1000);
            }
        }
    }

        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforiaWC() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //webcam name must be c310
            parameters.cameraName = hardwareMap.get(WebcamName.class, "C310");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

            //only detecting stones
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT//,
                    //LABEL_SECOND_ELEMENT
            );
        }
    }

