package org.firstinspires.ftc.teamcode.auto.auto_ai;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PixelDetection;
import org.firstinspires.ftc.teamcode.auto.AutoJava;
import org.firstinspires.ftc.teamcode.devices.CameraSpecs;
import org.firstinspires.ftc.teamcode.devices.PropSpecs;
import org.firstinspires.ftc.teamcode.devices.RobotSpecs;

import java.util.HashMap;

/**
 * Small test to see if the distance of the team prop from the camera can be calculated
 * and taken into affect for autonomous.
 *
 * <P>Must be tested on the blue center!<P>
 *
 * @author HarrisonFld
 */
@Autonomous(name = "Team Prop Distance Test", group = "Auto")
public class ObjectDistanceTest extends AutoJava {

    protected ObjectDistanceTest() {
        super(true); //Must be tested from the blue side
    }



    @Override
    public void runOpMode() {

        this.initMotors();
        this.initCamera();

        while (!isStarted()) {
            telemetry.addData("White percent of LCR mats:", pixelDetection.getLeftPercent() + " "
                    + pixelDetection.getCenterPercent() + " " + pixelDetection.getRightPercent());
            telemetry.addData("ROTATION1: ", pixelDetection.getPosition());
            telemetry.update();
        }


        double[] lowerBlueBounds = { 16,43,127,255 };
        double[] upperBlueBounds = { 102,168,255 };
        // Change later, calculate min width and height of team prop (recorded in camera res units) by getting camera picture and doing math involving
        // the ratio between the pixel size of the camera image and the resolution
        ColorLocationPipeline colorPipeline = new ColorLocationPipeline(lowerBlueBounds, upperBlueBounds, 10, 10);
        camera.setPipeline(colorPipeline);

        // Wait for pipeline to process. I don't think this is needed, but for now just keep it purely for early testing and development purposes
        sleep(3000);
        camera.closeCameraDevice();


        HashMap<Integer[], Integer[]> colorLocs = colorPipeline.getColorLocs();
        telemetry.addLine("Found locations involving blue:");
        colorLocs.forEach((cols, rows) -> {
            telemetry.addData("Columns:", cols);
            telemetry.addData("Rows:", rows);
        });
        telemetry.update();


        // THIS IS TEMPORARY
        Integer[] rows = colorLocs.entrySet().stream().toArray(Integer[]::new);
        double objectImageHeight = rows[1] - rows[0];


        PixelDetection.BackdropPosition position = pixelDetection.getPosition();

        //Must be tested on the center
        if (position == PixelDetection.BackdropPosition.CENTER) {

            double dist = calculateDistance(objectImageHeight); //Convert mm to inches
            telemetry.addLine(String.valueOf(dist));
            telemetry.update();

            moveBot(dist * Math.sin(RobotSpecs.cameraAngle), 1,0,0); //Move the horizontal distance of the robot to the prop
            //moveBot(dist,1,0,0); //Move the Straightforward distance of the camera to prop
//            lowerArm();
            
        } else {
            telemetry.addLine("Not Center: Team Prop detected " + position);
            telemetry.update();
        }
//        camera.closeCameraDevice();

    }

    /**
     *  Overrides the traditional moveBot, this one uses Millimeter as the distance the robot movements.
     */
    @Override
    protected void moveBot(double distMM, double forward, double pivot, double horizontal) {

        distMM /= 25.4; //Convert MM to inches
        distMM = (3.340 * distMM) - 2.356; //Convert to Robot Units

        super.moveBot(distMM, forward, pivot, horizontal);

    }

    /**
     * @param {double} units in pixels
     * @return distance in millimeters
     */
    private double calculateDistance(double objectImageHeight) {
        return (CameraSpecs.focalLength * objectImageHeight * CameraSpecs.cameraResHeight)
                / (PropSpecs.propHeight * CameraSpecs.sensorHeightMM);
    }


}
