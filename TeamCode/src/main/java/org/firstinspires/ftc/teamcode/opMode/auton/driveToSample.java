package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
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
@Autonomous (name = "g2Sample", group = "Autonomous Pathing")
public class driveToSample extends LinearOpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path first;
    private Path second;

    private PathChain test;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);

        first = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(12, -24, Point.CARTESIAN)));
        first.setTangentHeadingInterpolation();
        second = new Path(new BezierLine(new Point(12, -24, Point.CARTESIAN), new Point(16, -24, Point.CARTESIAN)));
        second.setConstantHeadingInterpolation(0);

        test = follower.pathBuilder().addPath(first).addPath(second).build();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

//      follower.followPath(first);
//      whatever you need to put inbetween ig
        follower.followPath(first, true);
        follower.update();
        do {
            follower.update();
        } while (follower.isBusy());
        follower.followPath(second, false);
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

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */


}
