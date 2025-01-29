package org.firstinspires.ftc.teamcode.piplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDet extends OpenCvPipeline {

    Telemetry telemetry;
    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat hierarchy = new Mat();

    // HSV thresholds for color detection
    public static double lowH = 0;
    public static double lowS = 104.8;
    public static double lowV = 86.4;
    public static double highH = 29.8;
    public static double highS = 255;
    public static double highV = 255;

    // Area range for selecting rectangles (in pixels)
    public static double minArea = 30000;
    public static double maxArea = 40000;

    public SampleDet(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        int frameCenterX = input.cols() / 2;
        int frameCenterY = input.rows() / 2;
        Point frameCenter = new Point(frameCenterX, frameCenterY);

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

                // Check if the rectangle is near the center
                if (Math.abs(rect.center.x - frameCenterX) < rect.size.width / 2 &&
                        Math.abs(rect.center.y - frameCenterY) < rect.size.height / 2) {

                    // Filter rectangles based on area
                    double area = rect.size.area();
                    if (area >= minArea && area <= maxArea) {
                        rotatedRects.add(rect);
                    }
                }
            }
        }

        // Step 5: Process the valid rectangle if found
        if (!rotatedRects.isEmpty()) {
            RotatedRect selectedRect = rotatedRects.get(0);

            Point[] vertices = new Point[4];
            selectedRect.points(vertices);

            // Draw the rectangle on the input frame
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Calculate angles relative to the Y-axis
            double angle1 = Math.atan2(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
            double angle2 = Math.atan2(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);

            // Convert to degrees
            angle1 = Math.toDegrees(angle1);
            angle2 = Math.toDegrees(angle2);

            // Normalize angles to -90° to 90° for proper sign representation
            if (angle1 > 90) angle1 -= 180;
            if (angle1 < -90) angle1 += 180;
            if (angle2 > 90) angle2 -= 180;
            if (angle2 < -90) angle2 += 180;

            // Define a tolerance for determining parallel edges
            double parallelTolerance = 15.0;

            // Highlight parallel edges if applicable
            if (Math.abs(angle1 - angle2) < parallelTolerance) {
                Imgproc.line(input, vertices[0], vertices[1], new Scalar(255, 0, 0), 3);
                Imgproc.line(input, vertices[2], vertices[3], new Scalar(255, 0, 0), 3);
            }

            // Draw angles on the screen
            String angleText = "Angle 1: " + String.format("%.2f", angle1) + "°";
            String angleText2 = "Angle 2: " + String.format("%.2f", angle2) + "°";
            Imgproc.putText(input, angleText, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(255, 255, 255), 2);
            Imgproc.putText(input, angleText2, new Point(10, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(255, 255, 255), 2);
        }

        // Draw the center point
        Imgproc.circle(input, frameCenter, 5, new Scalar(255, 0, 255), -1);

        return input;
    }
}
