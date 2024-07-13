package org.firstinspires.ftc.teamcode.auto.autoaitest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import

import org.firstinspires.ftc.teamcode.auto.AutoJava;

@Autonomous(name="Auto Run Robot", group="Auto")
public class AutoRunRobot extends AutoJava {

    protected AutoRunRobot(boolean blue) {
        super(blue);
    }

    @Override
    public void runOpMode() {

        this.initMotors();
        this.initCamera();

    }

    /*
    * Reference Documentation
    *
    * Methods below were made using documentation/tutorial(s) listed below:
    *   - [Get Distance via Opencv + Python](https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv)
    *       * Triangle Similarity Formula
    *       * Finding the Object
     */

    /**
    * Formula is derived from the Triangle Similarity formula for finding Focal Length
    *       F = (P x D) / W
    *   F=Focal length
    *   P=Pixel width of object in image
    *   D=Distance from camera to object
    *   W=Width known of object in image
    * <p>
    * Formula used is
    *       D = (W x F) / P
     */
    protected float getApproximateDistance(int objectWidth, float focalLength, int pixelWidth) {
        return ((objectWidth * focalLength) / pixelWidth);
    }

    /**
     * Unimplemented
     */
    protected void findObject() {
        return;
    }

}
