/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Drivetrain;
import org.firstinspires.ftc.teamcode.Assemblies.StoneScorer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Case 1", group = "Auton")
public class Case1 extends LinearOpMode {
    int liftValueA = -5400;
    int extendValueA = 2260;

    //names for cv objects
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "ASLOtt3/////AAABmbzx65TV2UrVqNnUS424xZpOs/Vw2BUdY1equY69euPD199BJxppV5RLjqwvUYyCtWjtNqI1CTL6Vlp5RvY5Cimm92p/ML2lDhM0GR/f2feDTFMgXLGPiEs7qStp839UrN8YNxDHbOdQHMCIlJeouhxOh9Y87rubm14L7RwrdvOyfo9v8o5ZyFqH33ap58P9xdmhIitqvU2nmVjieMZoTfGLuu0Fmuls+u3bHv5OfEcj8cEUlJ02sui0qdjfNcJIOPkNZUh822tYespPWEqEOLeOf3wXvy5skSQplg/1JOxPXdq8HUcCqeo25hL8iXkg1tlw12aCTLNkQli80Hw8Jiddnl1oKQe7cBziTEIXKmBW";

    Drivetrain d = new Drivetrain(this);
    StoneScorer ss = new StoneScorer(this);

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        d.init();
        ss.init();

        //init vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //init tfod
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //used to be 0.8
        tfodParameters.minimumConfidence = 0.42;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);



        tfod.activate();


        waitForStart();

        // findSkystone();
        List<Recognition> allRecognitions = null;
        for (int i = 0; i < 5; i++) {
            allRecognitions = tfod.getRecognitions();
            if (allRecognitions != null) {
                telemetry.addData("# Object Detected", allRecognitions.size());
                if (allRecognitions.size() == 2) {
                    break;
                }
                telemetry.update();


            }
            sleep(250);
        }

        int SkystoneX = -1;
        int Stone1X = -1;
        int Stone2X = -1;

        telemetry.addData("# Object Detected", allRecognitions.size());
        telemetry.update();

        for (Recognition recognition : allRecognitions) {
            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT) && SkystoneX==-1) {
                SkystoneX = (int) recognition.getLeft();
            } else if (Stone1X == -1) {
                Stone1X = (int) recognition.getLeft();
            } else {
                Stone2X = (int) recognition.getLeft();
            }
        }

        telemetry.addData("Skystone position: ", SkystoneX);
        telemetry.addData("stone 1: ", Stone1X);
        telemetry.addData("stone 2: ", Stone2X);

        sleep(3000000);

        // obtain skystone
        ss.setBlock(10, 2400);
        telemetry.addData("status:", "set block");
        telemetry.update();
        d.translate(Drivetrain.Direction.BACK, 12, 0.25);
        ss.intake(1, 10);
        telemetry.addData("status:", "set block");
        telemetry.update();

        d.translate(Drivetrain.Direction.LEFT, 12, 0.25);

        ss.setBlock(0, 0);
        telemetry.addData("status:", "reset to 0");
        telemetry.update();
        ss.intake(1, 10);

        ss.intake(1, 1);

        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        ss.extake(1, extendValueA, -1,1);

        // move right to position robot in front of second skystone
        d.translate(Drivetrain.Direction.RIGHT, 36, 0.25);

        // obtain skystone
        ss.setBlock(liftValueA, extendValueA);
        d.translate(Drivetrain.Direction.BACK, 12, 0.25);
        ss.intake(1, 1);

        // move left all the way into build zone
        d.translate(Drivetrain.Direction.LEFT, 36, 0.25);
        ss.extake(1, extendValueA, -1,1);

        // move right to park under bridge
        d.translate(Drivetrain.Direction.RIGHT, 36, 0.25);
    }
}
