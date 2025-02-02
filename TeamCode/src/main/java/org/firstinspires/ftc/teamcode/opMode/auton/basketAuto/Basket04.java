package org.firstinspires.ftc.teamcode.opMode.auton.basketAuto;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.fissionlib.command.AutoCommandMachine;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "0 + 4'", group = "!basket")
public class Basket04 extends OpMode {
    Timing.Timer startTimer = new Timing.Timer(250, TimeUnit.MILLISECONDS);

    Deposit deposit = new Deposit(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Intake diffy = new Intake(this);

    Follower follower;

    private final Pose startPose = new Pose(7, 103, Math.toRadians(270));
    private final Pose scorePose = new Pose(16, 118, Math.toRadians(315));
    private final Pose grab1Pose = new Pose(33, 120, Math.toRadians(0));
    private final Pose grab2Pose = new Pose(33, 130, Math.toRadians(0));
    private final Pose grab3Pose = new Pose(43, 120, Math.toRadians(90));
    private final Pose parkPose = new Pose(56, 100, Math.toRadians(90));

    private final Point parkControl = new Point(56, 130, Point.CARTESIAN);

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    // Path Commands
    Command preloadCommand = () -> follower.followPath(scorePreload);
    Command pickUp1Command = () -> follower.followPath(grabPickup1);
    Command pickUp2Command = () -> follower.followPath(grabPickup2);
    Command pickUp3Command = () -> follower.followPath(grabPickup3);
    Command s2Score = () -> follower.followPath(scorePickup1);
    Command s3Score = () -> follower.followPath(scorePickup2);
    Command s4Score = () -> follower.followPath(scorePickup3);
    Command parkCommand = () -> follower.followPath(park);
    // Deposit Commands
    Command grabDeposit = () -> deposit.closeClaw();
    Command releaseSample = () -> deposit.openClaw();
    Command basketPos = () -> deposit.basketPos();
    // Intake Commands
    Command openIntake = () -> diffy.openClaw();
    Command diffyTransfer = () -> diffy.diffyTransfer();
    Command diffyShift = () -> diffy.shiftClaw();

    CommandSequence preloadMove = new CommandSequence()
            .addCommand(this::mBasketSet)
            .addCommand(preloadCommand)
            .build();
    CommandSequence scoreBasket1 = new CommandSequence()
            .addCommand(basketPos)
            .addWaitCommand(0.2)
            .addCommand(releaseSample)
            .addWaitCommand(0.15)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(0.1)
            .addCommand(pickUp1Command)
            .build();
    CommandSequence grabSpike1 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.15)
            .addCommand(this::mIntakeRetract)
            .addWaitCommand(0.5)
            .addCommand(grabDeposit)
            .build();
    CommandSequence spike1Move = new CommandSequence()
            .addCommand(s2Score)
            .addWaitCommand(0.1)
            .addCommand(openIntake)
            .addCommand(this::mBasketSet)
            .build();
    CommandSequence scoreBasket2 = new CommandSequence()
            .addCommand(basketPos)
            .addWaitCommand(0.2)
            .addCommand(releaseSample)
            .addWaitCommand(0.15)
            .addCommand(this::mIntakeExtend)
            .addWaitCommand(0.1)
            .addCommand(pickUp2Command)
            .build();
    CommandSequence grabSpike2 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.15)
            .addCommand(this::mIntakeRetract)
            .addWaitCommand(0.5)
            .addCommand(grabDeposit)
            .build();
    CommandSequence spike2Move = new CommandSequence()
            .addCommand(s3Score)
            .addWaitCommand(0.1)
            .addCommand(openIntake)
            .addCommand(this::mBasketSet)
            .build();
    CommandSequence scoreBasket3 = new CommandSequence()
            .addCommand(basketPos)
            .addWaitCommand(0.2)
            .addCommand(releaseSample)
            .addWaitCommand(0.15)
            .addCommand(this::mIntakeExtend3)
            .addWaitCommand(0.1)
            .addCommand(pickUp3Command)
            .build();
    CommandSequence grabSpike3 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.15)
            .addCommand(this::mIntakeRetract)
            .addWaitCommand(0.5)
            .addCommand(grabDeposit)
            .build();
    CommandSequence spike3Move = new CommandSequence()
            .addCommand(s4Score)
            .addWaitCommand(0.1)
            .addCommand(openIntake)
            .addCommand(this::mBasketSet)
            .build();
    CommandSequence scoreBasket4 = new CommandSequence()
            .addCommand(basketPos)
            .addWaitCommand(0.2)
            .addCommand(releaseSample)
            .addWaitCommand(0.15)
            .addCommand(parkCommand)
            .addWaitCommand(0.2)
            .addCommand(this::mParkSet)
            .build();

    AutoCommandMachine runCommands = new AutoCommandMachine()
            .addCommandSequence(preloadMove)
            .addCommandSequence(scoreBasket1)
            .addCommandSequence(grabSpike1)
            .addCommandSequence(spike1Move)
            .addCommandSequence(scoreBasket2)
            .addCommandSequence(grabSpike2)
            .addCommandSequence(spike2Move)
            .addCommandSequence(scoreBasket3)
            .addCommandSequence(grabSpike3)
            .addCommandSequence(spike3Move)
            .addCommandSequence(scoreBasket4)
            .build();

    @Override
    public void init() {
        deposit.init(hardwareMap);
        slides.init(hardwareMap);
        diffy.init(hardwareMap);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        scorePreload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(scorePose)
                )
        );
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading());

        grabPickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(grab1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grab1Pose.getHeading()).build();

        grabPickup2 = follower.pathBuilder().addPath(
                    new BezierLine(
                                new Point(scorePose),
                                new Point(grab2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grab2Pose.getHeading()).build();

        grabPickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(grab3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grab3Pose.getHeading()).build();

        scorePickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(grab1Pose),
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), scorePose.getHeading()).build();

        scorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(grab2Pose),
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), scorePose.getHeading()).build();

        scorePickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(grab3Pose),
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), scorePose.getHeading()).build();

        park = new Path(
                new BezierCurve(
                        new Point(scorePose),
                        parkControl,
                        new Point(parkPose)
                )
        );
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        deposit.transferPos();
        deposit.openClaw();
        diffy.diffyTransfer();
        diffy.openClaw();
    }

    @Override
    public void start() {
        runCommands.run(false);
        startTimer.start();
        while (!startTimer.done()) {
            diffy.autoUpdate();
            slides.update();
            follower.update();
        }
    }

    @Override
    public void loop() {
        if (runCommands.hasCompleted()) requestOpModeStop();
        diffy.autoUpdate();
        slides.update();
        follower.update();
        runCommands.run(follower.isBusy() || !slides.isDone() || !runCommands.getCurrentCommand().hasCompleted);
    }

    public void mIntakeRetract() {
        diffy.diffyTransfer();
        diffy.shiftClaw();
        diffy.retractExtendy();
    }

    public void mIntakeExtend() {
        diffy.extendMax();
        diffy.diffyDown();
        diffy.openClaw();
        mSlideRest();
    }

    public void mIntakeExtend3() {
        diffy.extendMax();
        diffy.diffyDown();
        diffy.setDif_ROLL(90);
        diffy.openClaw();
        mSlideRest();
    }

    public void mBasketSet() {
        slides.setTarget(OuttakeSlides.HIGH_BASKET);
        diffy.diffyInterposed();
        diffy.openClaw();
        deposit.closeClaw();
        deposit.straightUpPos();
    }

    public void mSlideRest() {
        slides.intakePos();
        deposit.openClaw();
        deposit.transferPos();
    }

    public void mParkSet() {
        slides.ascent();
        deposit.openClaw();
        deposit.parkPos();
    }
}
