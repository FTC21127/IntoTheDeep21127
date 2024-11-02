package org.firstinspires.ftc.teamcode.opMode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class ParkAuton extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path first;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        first = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(5, -20, Point.CARTESIAN)));
        first.setConstantHeadingInterpolation(0);

        follower.followPath(first);
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()){
            stop();
        }
    }
}
