package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;


public class OpenCVTest extends OpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return new Mat(3,3,CvType.CV_8UC3);
    }
}
