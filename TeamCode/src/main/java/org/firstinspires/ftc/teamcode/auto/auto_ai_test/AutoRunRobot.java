package org.firstinspires.ftc.teamcode.auto.auto_ai_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.devices.CameraSpecs;
import org.firstinspires.ftc.teamcode.devices.PropSpecs;
import org.firstinspires.ftc.teamcode.devices.RobotSpecs;
import org.opencv.core.*;
import org.opencv.imgproc.*;

import org.firstinspires.ftc.teamcode.auto.AutoJava;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Comparator;
import java.util.List;

@Autonomous(name="Auto Run Robot", group="Auto")
public class AutoRunRobot extends AutoJava {

    public AutoRunRobot(boolean blue) {
        super(blue);
    }

    @Override
    public void runOpMode() {

        this.initMotors();
        initCamera(432, 240);

        float distanceToMoveMM = 0;

        while(!opModeIsActive()) {
            telemetry.addLine("Waiting For Start");
            distanceToMoveMM = beforeStart();
        }

        //Distance should never be 0 when testing
        if(distanceToMoveMM == 0) {
            throw new IllegalArgumentException("Distance was equal to 0");
        }

        //Code run after Start button has been pressed:
        afterStart(distanceToMoveMM);
    }

    protected float beforeStart() {
        RotatedRect foundObject = findObject(currentFrame);
        return getApproximateDistanceMM(PropSpecs.propWidth, CameraSpecs.focalLength, (int)foundObject.size.width);
    }

    protected void afterStart(float distance) {
        telemetry.addData("Distance approximated:", distance);

        //Horizontal distance calculated via the Pythagorean Theorem
        distance = (float)(Math.pow(distance, 2) - Math.pow(RobotSpecs.cameraHeightFromGround, 2));
        telemetry.addData("Horizontal distance approximated:", distance);

        moveBot((distance / 25.4), 0, 0, 1);
    }

    /**
     * Prevent use of previous camera code that uses prop color detection instead of edge detection
     */
    @Override
    @Deprecated
    protected void initCamera() {
        return;
    }


    protected Mat currentFrame;

    /**
     * Initialize the camera without the color detection for use in edge detection
     *<p>
     * Pass in streamWidth and streamHeight to set camera resolution
     */
    protected void initCamera(int streamWidth, int streamHeight) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        //Set pipeline to default pipeline that simply returns feed
        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                if (currentFrame != null) {
                    currentFrame = input; //Pass the current frame into the global Mat
                }
                return input;
            }
        });

        //Default Async Camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(streamWidth, streamHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error code:", errorCode);
                telemetry.update();
            }
        });


    }





    /*
    * Reference Documentation
    *
    * Methods below were made using documentation/tutorial(s) listed below:
    *   - [Get Distance via Opencv + Python](https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv)
    *       * Triangle Similarity Formula
    *       * Finding the Object
    *       * [Referencing Python OpenCV code to Java](https://stackoverflow.com/questions/48658449/opencv-convert-python-code-to-java)
    *   - [Example programs for different scenarios](https://github.com/bytedeco/javacv/tree/master/samples)
    *
    * @author HarrisonFld (2024)
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
    protected float getApproximateDistanceMM(float objectWidth, float focalLength, int pixelWidth) {
        return ((objectWidth * focalLength) / pixelWidth);
    }

    /**
     * Derived from the [Get Distance via Opencv + Python] listed above
     */
    protected RotatedRect findObject(Mat image) {

        //Gray the image, blur it, and then make it detect edges
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 35, 125);

        /*
        * Find largest contour/edge to determine object location
        *
        * WARNING: Only works if the object is big/close to the camera, for other scenarios a more
        * fine tuned algorithm will be needed
         */
        List<MatOfPoint> contours = new java.util.ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint largestContour = contours.stream().max(Comparator.comparingDouble(Imgproc::contourArea)).orElse(null);

        if (largestContour == null) {
            throw new RuntimeException("No contours found");
        }

        //Compute the bounding box of the largest contour
        return Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
    }





}
