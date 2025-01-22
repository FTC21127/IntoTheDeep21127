package org.firstinspires.ftc.teamcode.opMode.auton.specimen;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.fissionlib.command.AutoCommandMachine;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "4 + 0'", group = "!specimen", preselectTeleOp = "RedTele")
public class Specimen40 extends OpMode {

    Timing.Timer startTimer = new Timing.Timer(1000, TimeUnit.SECONDS);

    Deposit deposit = new Deposit(this);
    Drivetrain drivetrain = new Drivetrain(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Intake diffy = new Intake(this);

    Follower follower;
    Path preload;
    PathChain pickUp1, hp1, pickUp2, hp2, pickUp3, hp3, speci2Set, speci2Score, speci3Set, speci3Score, speci4Set, speci4Score;

    private final Pose startPose = new Pose(7.48, 55, Math.toRadians(180));
    private final Pose preloadPose = new Pose(39, 61.5, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 66, Math.toRadians(180));
    private final Pose grabPose1 = new Pose(35, 38, Math.toRadians(-45));
    private final Pose grabPose2 = new Pose(35, 24, Math.toRadians(-45));
    private final Pose grabPose3 = new Pose(35, 18, Math.toRadians(-45));
    private final Pose hpPose = new Pose(26, 30, Math.toRadians(-135));
    private final Pose pickUpPose = new Pose(13, 40, Math.toRadians(-90));

    private final Point gSControl = new Point(24, 53, Point.CARTESIAN);
    private final Point sControl1 = new Point(20, 45, Point.CARTESIAN);
    private final Point sControl2 = new Point(40, 60, Point.CARTESIAN);

    // Path Commands
    Command preloadCommand = () -> follower.followPath(preload);
    Command pickUp1Command = () -> follower.followPath(pickUp1);
    Command pickUp2Command = () -> follower.followPath(pickUp2);
    Command pickUp3Command = () -> follower.followPath(pickUp3);
    Command hp1Command = () -> follower.followPath(hp1);
    Command hp2Command = () -> follower.followPath(hp2);
    Command hp3Command = () -> follower.followPath(hp3);
    Command s2Set = () -> follower.followPath(speci2Set);
    Command s3Set = () -> follower.followPath(speci3Set);
    Command s4Set = () -> follower.followPath(speci4Set);
    Command s2Score = () -> follower.followPath(speci2Score);
    Command s3Score = () -> follower.followPath(speci3Score);
    Command s4Score = () -> follower.followPath(speci4Score);
    // Deposit Commands
    Command grabDeposit = () -> deposit.closeClaw();
    Command releaseSpecimen = () -> deposit.openClaw();
    Command lockSpecimen = () -> deposit.specimenScorePos();
    // Intake Commands
    Command grabIntake = () -> diffy.closeClaw();
    Command releaseSample = () -> diffy.openClaw();
    Command diffyInterpose = () -> diffy.diffyInterposed();
    Command diffyRetract = () -> diffy.retractExtendy();
    Command diffyTransfer = () -> diffy.diffyTransfer();
    Command difftShift = () -> diffy.shiftClaw();

    CommandSequence scorePreload = new CommandSequence()
            .addCommand(this::mSpecimenSet)
            .addCommand(preloadCommand)
            .build();
    CommandSequence grabSample1 = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.2)
            .addCommand(releaseSpecimen)
            .addCommand(pickUp1Command)
            .addWaitCommand(.5)
            .addCommand(this::mSampleGrab)
            .addCommand(this::mSlideRest)
            .build();
    CommandSequence human1 = new CommandSequence()
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp1Command)
            .build();
    CommandSequence grabSample2 = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(pickUp2Command)
            .build();
    CommandSequence human2 = new CommandSequence()
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp2Command)
            .build();
    CommandSequence grabSample3 = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(pickUp3Command)
            .build();
    CommandSequence human3 = new CommandSequence()
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp3Command)
            .build();
    CommandSequence speci2Grab = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(s2Set)
            .addCommand(this::mRetractSet)
            .addWaitCommand(2)
            .addCommand(this::mSpecimenPickup)
            .addWaitCommand(.4)
            .build();
    CommandSequence scoreSpeci2 = new CommandSequence()
            .addCommand(difftShift)
            .addWaitCommand(.2)
            .addCommand(diffyInterpose)
            .addCommand(diffyRetract)
            .addWaitCommand(.3)
            .addCommand(s2Score)
            .addCommand(grabIntake)
            .addCommand(diffyTransfer)
            .addWaitCommand(.2)
            .addCommand(grabDeposit)
            .addWaitCommand(.1)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence speci3Grab = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.2)
            .addCommand(releaseSpecimen)
            .addCommand(s3Set)
            .addCommand(this::mSpecimenPickup)
            .addWaitCommand(.4)
            .build();
    CommandSequence scoreSpeci3 = new CommandSequence()
            .addCommand(difftShift)
            .addWaitCommand(.2)
            .addCommand(diffyInterpose)
            .addCommand(diffyRetract)
            .addWaitCommand(.3)
            .addCommand(s3Score)
            .addCommand(grabIntake)
            .addCommand(diffyTransfer)
            .addWaitCommand(.2)
            .addCommand(grabDeposit)
            .addWaitCommand(.1)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence speci4Grab = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.2)
            .addCommand(releaseSpecimen)
            .addCommand(s4Set)
            .addCommand(this::mSpecimenPickup)
            .addWaitCommand(.4)
            .build();
    CommandSequence scoreSpeci4 = new CommandSequence()
            .addCommand(difftShift)
            .addWaitCommand(.2)
            .addCommand(diffyInterpose)
            .addCommand(diffyRetract)
            .addWaitCommand(.3)
            .addCommand(s4Score)
            .addCommand(grabIntake)
            .addCommand(diffyTransfer)
            .addWaitCommand(.2)
            .addCommand(grabDeposit)
            .addWaitCommand(.1)
            .addCommand(this::mSpecimenSet)
            .build();

    AutoCommandMachine runCommands = new AutoCommandMachine()
            .addCommandSequence(scorePreload)
            .addCommandSequence(grabSample1)
            .addCommandSequence(human1)
            .addCommandSequence(grabSample2)
            .addCommandSequence(human2)
            .addCommandSequence(grabSample3)
            .addCommandSequence(human3)
            .addCommandSequence(speci2Grab)
            .addCommandSequence(scoreSpeci2)
            .addCommandSequence(speci3Grab)
            .addCommandSequence(scoreSpeci3)
            .addCommandSequence(speci3Grab)
            .addCommandSequence(scoreSpeci3)
            .addCommandSequence(speci4Grab)
            .addCommandSequence(scoreSpeci4)
            .build();

    @Override
    public void init() {
        deposit.init(hardwareMap);
        drivetrain.init(hardwareMap);
        slides.init(hardwareMap);
        diffy.init(hardwareMap);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        preload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)
                )
        );
        preload.setConstantHeadingInterpolation(Math.toRadians(180));

        pickUp1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(preloadPose),
                                gSControl,
                                new Point(grabPose1)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), grabPose1.getHeading()).build();

        hp1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(grabPose1),
                                new Point(hpPose)))
                .setLinearHeadingInterpolation(grabPose1.getHeading(), hpPose.getHeading()).build();

        pickUp2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(hpPose),
                                new Point(grabPose2)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), grabPose2.getHeading()).build();

        hp2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(grabPose2),
                                new Point(hpPose)))
                .setLinearHeadingInterpolation(grabPose2.getHeading(), hpPose.getHeading()).build();

        pickUp3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(hpPose),
                                new Point(grabPose3)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), grabPose3.getHeading()).build();

        hp3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(grabPose3),
                                new Point(hpPose)))
                .setLinearHeadingInterpolation(grabPose3.getHeading(), hpPose.getHeading()).build();

        speci2Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(hpPose),
                                new Point(pickUpPose)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), pickUpPose.getHeading()).build();

        speci2Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose),
                                sControl1,
                                sControl2,
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading()).build();

        speci3Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(hpPose),
                                new Point(pickUpPose)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), pickUpPose.getHeading()).build();

        speci3Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose),
                                sControl1,
                                sControl2,
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading()).build();

        speci4Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(hpPose),
                                new Point(pickUpPose)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), pickUpPose.getHeading()).build();

        speci4Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose),
                                sControl1,
                                sControl2,
                                new Point(scorePose)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading()).build();


        deposit.transferPos();
        deposit.openClaw();
        diffy.diffyTransfer();
        diffy.openClaw();
    }

    @Override
    public void init_loop() {
        if (gamepad1.cross) {
            deposit.closeClaw();
        }
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
        diffy.autoUpdate();
        slides.update();
        follower.update();
        runCommands.run(follower.isBusy() || !slides.isDone());
    }

    public void mSampleGrab() {
        diffy.extendNeutral();
        diffy.diffyDown();
        diffy.setDif_ROLL(45);
        diffy.openClaw();
    }

    public void mRetractSet() {
        diffy.retractExtendy();
        diffy.diffyInterposed();
        diffy.openClaw();
    }

    public void mSpecimenPickup() {
        diffy.diffyDown();
        diffy.extendMax();
        diffy.centerRoll();
        diffy.openClaw();
    }

    public void mITransfer() {
        diffy.openClaw();
        diffy.diffyInterposed();
    }

    public void mSpecimenSet() {
        slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET);
        deposit.closeClaw();
        deposit.specimenPos();
        diffy.openClaw();
        diffy.diffyInterposed();
    }

    public void mSlideRest() {
        slides.intakePos();
        deposit.openClaw();
        deposit.transferPos();
    }
}

//.addPath(
//        // Line 1
//        new BezierLine(
//          new Point(7.500, 55.000, Point.CARTESIAN),
//          new Point(39.000, 61.500, Point.CARTESIAN)
//        )
//      )
//      .setConstantHeadingInterpolation(Math.toRadians(180))
//      .addPath(
//        // Line 2
//        new BezierCurve(
//          new Point(39.000, 61.500, Point.CARTESIAN),
//          new Point(24.000, 53.000, Point.CARTESIAN),
//          new Point(35.000, 38.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
//      .addPath(
//        // Line 3
//        new BezierLine(
//          new Point(35.000, 38.000, Point.CARTESIAN),
//          new Point(26.000, 30.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-135))
//      .addPath(
//        // Line 4
//        new BezierLine(
//          new Point(26.000, 30.000, Point.CARTESIAN),
//          new Point(35.000, 24.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(315))
//      .addPath(
//        // Line 5
//        new BezierLine(
//          new Point(35.000, 24.000, Point.CARTESIAN),
//          new Point(26.000, 30.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-135))
//      .addPath(
//        // Line 6
//        new BezierLine(
//          new Point(26.000, 30.000, Point.CARTESIAN),
//          new Point(35.000, 14.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(315))
//      .addPath(
//        // Line 7
//        new BezierLine(
//          new Point(35.000, 14.000, Point.CARTESIAN),
//          new Point(26.000, 30.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-135))
//      .addPath(
//        // Line 8
//        new BezierLine(
//          new Point(26.000, 30.000, Point.CARTESIAN),
//          new Point(13.000, 40.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(270))
//      .addPath(
//        // Line 9
//        new BezierCurve(
//          new Point(13.000, 40.000, Point.CARTESIAN),
//          new Point(17.000, 50.000, Point.CARTESIAN),
//          new Point(40.000, 62.000, Point.CARTESIAN),
//          new Point(39.000, 66.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
//      .addPath(
//        // Line 10
//        new BezierLine(
//          new Point(39.000, 66.000, Point.CARTESIAN),
//          new Point(13.000, 40.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
//      .addPath(
//        // Line 11
//        new BezierCurve(
//          new Point(13.000, 40.000, Point.CARTESIAN),
//          new Point(17.000, 50.000, Point.CARTESIAN),
//          new Point(40.000, 62.000, Point.CARTESIAN),
//          new Point(39.000, 66.000, Point.CARTESIAN)
//        )
//      )
//      .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180));