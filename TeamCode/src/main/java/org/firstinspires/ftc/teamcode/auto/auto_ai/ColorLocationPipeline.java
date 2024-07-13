package org.firstinspires.ftc.teamcode.auto.auto_ai;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;

public class ColorLocationPipeline extends OpenCvPipeline {


    private double[] rgbMin;
    private double[] rgbMax;
    private int minWidth;
    private int minHeight;

    // Map that associates found columns containing color with the rows that also contain the color.
    // If used correctly, you can create a box containing the object you would like to find
    private HashMap<Integer[], Integer[]> colorLocs = new HashMap<>();


    public ColorLocationPipeline(double[] rgbMin, double[] rgbMax, int minWidth, int minHeight) {
        this.rgbMin = rgbMin;
        this.rgbMax = rgbMax;
        this.minWidth = minWidth;
        this.minHeight = minHeight;
    }


    @Override
    public Mat processFrame(Mat input) {

        HashMap<Integer[], Integer[]> colorLocs = new HashMap<>();


        int currLeftCol = -1;
        int currRightCol = -1;
        int currLowestRow = -1;
        int currHighestRow = -1;
        boolean iteratingCols = false;
        for (int col = 0; col < input.cols(); col++) {

            if (
                (currLeftCol >= 0 && currRightCol >= 0) &&
                ((currRightCol - currLeftCol >= minWidth) && (currHighestRow - currLowestRow >= minHeight))
               ) {
                colorLocs.put(new Integer[]{ currLeftCol, currRightCol }, new Integer[]{ currLowestRow, currHighestRow });
            }
            currLeftCol = -1;
            currRightCol = -1;
            currLowestRow = -1;
            currHighestRow = -1;



            boolean iteratingRows = false;
            for (int row = 0; row < input.rows(); row++) {


                double[] pixelRGB = input.get(col, row);
                boolean pixelInRange = true;
                for (int rgbIndex = 0; rgbIndex < pixelRGB.length; rgbIndex++) {

                    double minVal = rgbMin[rgbIndex];
                    double maxVal = rgbMax[rgbIndex];
                    double pixelVal = pixelRGB[rgbIndex];

                    boolean rgbInRange = (pixelVal >= minVal && pixelVal <= maxVal);
                    if (!rgbInRange) {
                        pixelInRange = false;
                        break;
                    }
                }

                if (pixelInRange) {
                    if (row < currLowestRow) {
                        currLowestRow = row;
                    }
                    iteratingRows = true;
                } else {
                    if (iteratingRows) {
                        if (row > currHighestRow) {
                            currHighestRow = row - 1;
                        }
                        iteratingRows = false;
                        break;
                    }
                }
            }


            if (currLowestRow == -1 || currHighestRow == -1) {
                if (iteratingCols) {
                    currRightCol = col - 1;
                    iteratingCols = false;
                }
                continue;
            } else {
                currLeftCol = col;
                iteratingCols = true;
            }

        }


        this.colorLocs = colorLocs;
        input.release();
        return input;

    }


    public HashMap<Integer[], Integer[]> getColorLocs() {
        return this.colorLocs;
    }
}
