package org.firstinspires.ftc.teamcode.piplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class sampleDetectionTuning extends OpenCvPipeline {

    Telemetry telemetry;
    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat hierarchy = new Mat();

    // HSV thresholds for color detection
    public static double lowH = 0;
    public static double lowS =104.8;
    public static double lowV = 86.4;
    public static double highH = 29.8;
    public static double highS = 255;
    public static double highV = 255;

    // Area range for selecting rectangles (in pixels)
    public static double minArea = 500; // Minimum area threshold
    public static double maxArea = 5000; // Maximum area threshold

    public sampleDetectionTuning(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Step 1: Convert input to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Step 2: Apply color threshold to isolate target color
        Scalar lowerBound = new Scalar(lowH, lowS, lowV);
        Scalar upperBound = new Scalar(highH, highS, highV);
        Core.inRange(hsvMat, lowerBound, upperBound, binaryMat);

        // Step 3: Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(binaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Step 4: Draw contours on the input frame
        for (MatOfPoint contour : contours) {
            // Calculate the area of the contour
            double area = Imgproc.contourArea(contour);

            // Only draw contours within the area range
            if (area >= minArea) {
                // Draw the contour (green color, thickness 2)
                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);

                // Optionally, draw a bounding rectangle around the contour
                Rect boundingRect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(input, boundingRect, new Scalar(255, 0, 0), 2);

                // Telemetry data for debugging
                telemetry.addData("Contour Area", area);
                telemetry.addData("Bounding Rect", "x: %d, y: %d, w: %d, h: %d",
                        boundingRect.x, boundingRect.y, boundingRect.width, boundingRect.height);
            }
        }

        // Update telemetry
        telemetry.update();

        // Return the annotated input frame
        return input;
    }
}