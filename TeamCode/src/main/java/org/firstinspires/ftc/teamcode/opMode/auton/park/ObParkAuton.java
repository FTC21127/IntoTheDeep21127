package org.firstinspires.ftc.teamcode.opMode.auton.park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Observation Park", group = "!park", preselectTeleOp = "Robot")
public class ObParkAuton extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path first;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        first = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(2, -24, Point.CARTESIAN)));
        first.setConstantHeadingInterpolation(0);

        follower.followPath(first, true);
    }

    @Override
    public void start() {
        do {
            follower.update();
        } while (follower.isBusy());
    }

    @Override
    public void loop() {

    }
}
