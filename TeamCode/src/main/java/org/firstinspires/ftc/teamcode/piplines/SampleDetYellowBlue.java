package org.firstinspires.ftc.teamcode.piplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetYellowBlue extends OpenCvPipeline {

    Telemetry telemetry;
    private Mat hsvMat = new Mat();
    private Mat binaryMat1 = new Mat();
    private Mat binaryMat2 = new Mat();
    private Mat combinedBinaryMat = new Mat();
    private Mat hierarchy = new Mat();

    // First color HSV thresholds
    public static double YlowH = 0, YlowS = 104.8, YlowV = 86.4;
    public static double YhighH = 29.8, YhighS = 255, YhighV = 255;

    // Second color HSV thresholds
    public static double BlowH = 102, BlowS = 22.7, BlowV = 22.7;
    public static double BhighH = 120.4, BhighS = 255, BhighV = 255;

    // Area range for selecting rectangles (in pixels)
    public static double minArea = 5000;
    public static double maxArea = 6600;

    public SampleDetYellowBlue(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        int frameCenterX = input.cols() / 2;
        int frameCenterY = input.rows() / 2;
        Point frameCenter = new Point(frameCenterX, frameCenterY);

        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold for first color
        Scalar lowerBound1 = new Scalar(YlowH, YlowS, YlowV);
        Scalar upperBound1 = new Scalar(YhighH, YhighS, YhighV);
        Core.inRange(hsvMat, lowerBound1, upperBound1, binaryMat1);

        // Threshold for second color
        Scalar lowerBound2 = new Scalar(BlowH, BlowS, BlowV);
        Scalar upperBound2 = new Scalar(BhighH, BhighS, BhighV);
        Core.inRange(hsvMat, lowerBound2, upperBound2, binaryMat2);

        // Combine both binary masks
        Core.bitwise_or(binaryMat1, binaryMat2, combinedBinaryMat);

        // Find contours in the combined binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Analyze contours to find rotated rectangles
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

        // Process detected rectangles
        for (RotatedRect selectedRect : rotatedRects) {
            Point[] vertices = new Point[4];
            selectedRect.points(vertices);

            // Draw the rectangle on the input frame
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Identify the two side lengths of the rectangle
            double side1 = Math.hypot(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
            double side2 = Math.hypot(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);

            // Determine which side is longer
            boolean firstSideIsLonger = side1 > side2;

            // Compute angles relative to the Y-axis
            double longerAngle, shorterAngle;

            if (firstSideIsLonger) {
                longerAngle = Math.atan2(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
                shorterAngle = Math.atan2(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);
            } else {
                longerAngle = Math.atan2(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);
                shorterAngle = Math.atan2(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
            }

            // Convert to degrees
            longerAngle = Math.toDegrees(longerAngle);
            shorterAngle = Math.toDegrees(shorterAngle);

            // Normalize angles to -90° to 90°
            if (longerAngle > 90) longerAngle -= 180;
            if (longerAngle < -90) longerAngle += 180;
            if (shorterAngle > 90) shorterAngle -= 180;
            if (shorterAngle < -90) shorterAngle += 180;

            // Display the angles
            String angleText = "Angle: " + String.format("%.2f", longerAngle);
            Imgproc.putText(input, angleText, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(255, 255, 255), 2);
        }

        // Draw the center point
        Imgproc.circle(input, frameCenter, 5, new Scalar(255, 0, 255), -1);

        return input;
    }
}
