package org.firstinspires.ftc.teamcode.opMode.auton;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "0 + 1 Low", group = "!Sample")
public class BucketParkAuton extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private Path first, second, third, fourth, five;

    private Intake claw = new Intake(this);
    private OuttakeSlides slides = new OuttakeSlides(this);
    private Deposit deposit = new Deposit(this);

    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    // Slide Commands
    private final Command slidesIntake = () -> slides.intakePos();
    private final Command slideRest = () -> slides.restPos();
    private final Command slideUp = () -> slides.setTarget(OuttakeSlides.LOW_BASKET);
    private final Command lockSpecimen = () -> slides.lock();
    // Deposit Commands
    private final Command depositPos = () -> deposit.depositPos();
    private final Command grabTransfer = () -> deposit.transferPos();
    private final Command basketPos = () -> deposit.basketPos();
    private final Command outtakeRelease = () -> deposit.openClaw();
    private final Command outtakeGrab = () -> deposit.closeClaw();
    // Intake Commands
    private final Command clawGrab = () -> claw.closeClaw();
    private final Command clawOpen = () -> claw.openClaw();
    private final Command dropV4b = () -> claw.barDown();
    private final Command neutralV4b = () -> claw.barNeutral();
    private final Command transferV4b = () -> claw.barTransfer();
    private final Command climbV4b = () -> claw.barFold();

    private CommandSequence depositSequence = new CommandSequence()
            .addCommand(outtakeGrab)
            .addWaitCommand(.5)
            .addCommand(slideUp)
            .addWaitCommand(0.2)
            .addCommand(basketPos)
            .build();

    private CommandSequence depositSample = new CommandSequence()
            .addCommand(basketPos)
            .addCommand(outtakeRelease)
            .addWaitCommand(.2)
            .addCommand(grabTransfer)
            .addWaitCommand(1)
            .addCommand(outtakeRelease)
            .addCommand(slidesIntake)
            .build();

    @Override
    public void init() {
        claw.init(hardwareMap);
        slides.init(hardwareMap);
        deposit.init(hardwareMap);
        follower = new Follower(hardwareMap);
        first = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(10, 10, Point.CARTESIAN)));
        first.setConstantHeadingInterpolation(0);
        
        second = new Path(new BezierLine(first.getLastControlPoint(), new Point(11, 11, Point.CARTESIAN)));
        second.setConstantHeadingInterpolation(Math.toRadians(360-45));

        third = new Path(new BezierLine(second.getLastControlPoint(), new Point(5, 16, Point.CARTESIAN)));
        third.setConstantHeadingInterpolation(Math.toRadians(360-45));

        fourth = new Path(new BezierLine(third.getLastControlPoint(), new Point(49, 10, Point.CARTESIAN)));
        fourth.setConstantHeadingInterpolation(Math.toRadians(90));

        five = new Path(new BezierLine(fourth.getLastControlPoint(), new Point(49, 1, Point.CARTESIAN)));
        five.setConstantHeadingInterpolation(Math.toRadians(90));
        follower.followPath(first, true);
        claw.barNeutral();
    }

    @Override
    public void start() {
        deposit.basketPos();
        slides.restPos();
        deposit.openClaw();
        do {
            slides.update();
            follower.update();
        } while (follower.isBusy());
        follower.followPath(second);
        deposit.transferPos();
        deposit.openClaw();
        slides.intakePos();
        do {
            slides.update();
            follower.update();
        } while (follower.isBusy());
        depositSequence.trigger();
        follower.followPath(third);
        do {
            slides.update();
            follower.update();
        } while (follower.isBusy());
        depositSample.trigger();
        timer.start();
        while (!timer.done()){}
        follower.followPath(fourth);
        do {
            slides.update();
            follower.update();
        } while (follower.isBusy());
        follower.followPath(five);
        do {
            slides.update();
            follower.update();
        } while (follower.isBusy());
    }

    @Override
    public void loop() {

    }
}
