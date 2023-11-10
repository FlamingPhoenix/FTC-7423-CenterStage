/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "PropDetector", group = "Concept")
public class PropDetector extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private stretchProcessor stretchProcessor;
    private TfodProcessor tfod;
    PtzControl ptzControl;
    private VisionPortal visionPortal;
    String[] poss = {"left","center","right","none"};

    @Override
    public void runOpMode() {
        initTfod();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        int pos = getPos();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(gamepad1.a){
                    pos = getPos();
                }
                telemetry.addData("side",poss[pos]);
                telemetry.update();

                sleep(20);
            }
        }

        visionPortal.close();

    }   // end runOpMode()
    private void initTfod() {

        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_ASSET).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 640));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.7f);
    }
    private int getDetection() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        int pos = 3;
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size()>0){
            Recognition recognition = currentRecognitions.get(0);
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if(x>=0 && x<=213){
                pos = 0;
            }else if(x>213 && x<=426){
                pos = 1;
            }else if(x>426 && x<=640){
                pos = 2;
            }
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("side",poss[pos]);
        }   // end for() loop
        return pos;
    }   // end method telemetryTfod()

    private int getPos(){
        int[] counts = new int[4];
        for(int i = 0; i<10; i++){
            counts[getDetection()]++;  //get 10 detections & note the outputs
            sleep(20);
        }
        int max = 0;
        int maxIndex = 0;
        for(int i = 0; i<4; i++){
            if(counts[i]>max){
                max = counts[i]; // get the max count from the detections -- this is the most common output
                maxIndex = i;
            }
        }
        return maxIndex;
    }

}   // end class
