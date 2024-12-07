package org.firstinspires.ftc.teamcode.opMode.auton.park;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Ascent Park", group = "#park", preselectTeleOp = "Robot")
public class AcParkAuton extends OpMode {

    private Follower follower;
    private Timing.Timer timer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    private Path first, second;

    private Intake claw = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);

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
        slides.setTarget(OuttakeSlides.REST_POS + 400);
        timer.start();
        while (!timer.done()){
            slides.update();
        }
        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
