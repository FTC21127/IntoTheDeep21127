package org.firstinspires.ftc.teamcode.opMode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
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

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        follower = new Follower(hardwareMap);

        first = new Path(new BezierLine(new Point(0, 49, Point.CARTESIAN), new Point(24, 16, Point.CARTESIAN)));
        first.setTangentHeadingInterpolation();
        second = new Path(new BezierLine(new Point(24, 16, Point.CARTESIAN), new Point(31, 16, Point.CARTESIAN)));
        second.setConstantHeadingInterpolation(0);


        follower.followPath(first);
        // whatever you need to put inbetween ig
        follower.followPath(second);
        //pic up shit

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.update();
    }
}
