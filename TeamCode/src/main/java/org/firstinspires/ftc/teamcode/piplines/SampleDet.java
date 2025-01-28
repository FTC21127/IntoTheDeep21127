package org.firstinspires.ftc.teamcode.piplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class SampleDet extends OpenCvPipeline {

    Telemetry telemetry;
    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat hierarchy = new Mat();

    // HSV thresholds for color detection
    public static double lowH = 20;
    public static double lowS = 100;
    public static double lowV = 100;
    public static double highH = 30;
    public static double highS = 255;
    public static double highV = 255;

    // Area range for selecting rectangles (in pixels)
    public static double minArea = 500; // Minimum area threshold
    public static double maxArea = 5000; // Maximum area threshold

    public SampleDet(Telemetry telemetry) {
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

        // Step 4: Analyze contours to find rotated rectangles
        List<RotatedRect> rotatedRects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 100) { // Filter small contours
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);

                // Filter rectangles based on area
                double area = rect.size.area();
                if (area >= minArea && area <= maxArea) {
                    rotatedRects.add(rect);
                }
            }
        }

        // Step 5: Find the two longest parallel edges
        if (rotatedRects.size() > 0) {
            RotatedRect selectedRect = rotatedRects.get(0); // Select the first valid rectangle

            Point[] vertices = new Point[4];
            selectedRect.points(vertices);

            // Draw the rectangle on the input frame
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Calculate the angles of the edges
            double angle1 = Math.atan2(vertices[1].y - vertices[0].y, vertices[1].x - vertices[0].x);
            double angle2 = Math.atan2(vertices[2].y - vertices[1].y, vertices[2].x - vertices[1].x);

            // Convert angles to degrees
            angle1 = Math.toDegrees(angle1);
            angle2 = Math.toDegrees(angle2);

            // Define a tolerance for determining parallel edges
            double parallelTolerance = 15.0; // Adjust as needed

            // Highlight the longest parallel lines (edges with smallest difference in angle)
            if (Math.abs(angle1 - angle2) < parallelTolerance) {
                Imgproc.line(input, vertices[0], vertices[1], new Scalar(255, 0, 0), 3);
                Imgproc.line(input, vertices[2], vertices[3], new Scalar(255, 0, 0), 3);
            }

            // Draw angles on the screen
            String angleText = "Angle 1: " + String.format("%.2f", angle1) + "° | Angle 2: " + String.format("%.2f", angle2) + "°";
            Imgproc.putText(input, angleText, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 255), 2);
        }

        // Return the annotated input frame
        return input;
    }
}
