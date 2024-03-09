package org.firstinspires.ftc.teamcode.AI;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class examplePipeline extends OpenCvPipeline {

    public Mat mat;
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        return mat;
    }
}
