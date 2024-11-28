package org.firstinspires.ftc.teamcode.opMode.auton.park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Ascent Park", group = "!park", preselectTeleOp = "Robot")
public class AcParkAuton extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path first, second;

    private Intake claw = new Intake(this);

    @Override
    public void init() {
        claw.init(hardwareMap);
        follower = new Follower(hardwareMap);

        first = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(44, 7, Point.CARTESIAN)));
        first.setConstantHeadingInterpolation(0);

        second = new Path(new BezierLine(first.getLastControlPoint(), new Point(48, -1, Point.CARTESIAN)));
        second.setConstantHeadingInterpolation(Math.toRadians(90));
        follower.followPath(first, true);
    }

    @Override
    public void start() {
        do {
            follower.update();
        } while (follower.isBusy());
        claw.barNeutral();
        follower.followPath(second);
        do {
            follower.update();
        } while (follower.isBusy());
    }

    @Override
    public void loop() {

    }
}
