package org.firstinspires.ftc.teamcode.opMode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Pedropath test", group = "dev")
public class LocalTest extends LinearOpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private PathChain path;
    
    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);

        path = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(38.350, 62.225, Point.CARTESIAN),
                        new Point(2.632, 42.674, Point.CARTESIAN),
                        new Point(93.619, 32.898, Point.CARTESIAN),
                        new Point(61.097, 22.747, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(181), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(61.097, 22.747, Point.CARTESIAN),
                        new Point(16.731, 23.311, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(16.731, 23.311, Point.CARTESIAN),
                        new Point(68.616, 32.522, Point.CARTESIAN),
                        new Point(58.277, 12.595, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(58.277, 12, Point.CARTESIAN),
                        new Point(15.791, 12, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(15.791, 12, Point.CARTESIAN),
                        new Point(18.047, 35.718, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.setPose(new Pose(38.350, 62.225, Math.toRadians(181)));

        waitForStart();

        follower.followPath(path, true);
        follower.update();
        do {
            follower.update();
            telemetryA.addData("is busy: ", true);
            telemetryA.update();
        } while (follower.isBusy());
        telemetryA.addData("is busy: ", "NO");
        telemetryA.update();

        //pic up shit

    }

    /*
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */


}
