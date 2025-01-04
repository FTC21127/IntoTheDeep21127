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
@Autonomous(name = "Pedropath test", group = "dev")
public class LocalTest extends LinearOpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private PathChain path;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);

        path = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(8.100, 37.000, Point.CARTESIAN),
                        new Point(62.000, 37.000, Point.CARTESIAN),
                        new Point(62.000, 29.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(62.000, 29.000, Point.CARTESIAN),
                        new Point(50.000, 24.000, Point.CARTESIAN),
                        new Point(24.000, 24.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(24.000, 24.000, Point.CARTESIAN),
                        new Point(76.000, 30.000, Point.CARTESIAN),
                        new Point(76.000, 10.000, Point.CARTESIAN),
                        new Point(24.000, 13.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.setPose(new Pose(8.1, 37, Math.toRadians(0)));

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
