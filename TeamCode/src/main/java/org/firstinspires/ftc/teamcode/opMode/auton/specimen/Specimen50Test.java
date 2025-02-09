package org.firstinspires.ftc.teamcode.opMode.auton.specimen;

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

@Autonomous(name = "5 + 0", group = "!specimen", preselectTeleOp = "RedTele")
public class Specimen50Test extends OpMode {

    Timing.Timer startTimer = new Timing.Timer(250, TimeUnit.MILLISECONDS);

    Deposit deposit = new Deposit(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Intake diffy = new Intake(this);

    Follower follower;
    Path preload, park;
    PathChain pickUp1, hp1, pickUp2, hp2, pickUp3, hp3, speci2Set, speci2Score, speci3Set, speci3Score, speci4Set, speci4Score, speci5Set, speci5Score;

    private final Pose startPose = new Pose(7.48, 55, Math.toRadians(180));
    private final Pose preloadPose = new Pose(37, 73, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 69, Math.toRadians(180));
    private final Pose grabPose1 = new Pose(33.5, 33.8, Math.toRadians(-48));
    private final Pose grabPose2 = new Pose(31, 24., Math.toRadians(-48));
    private final Pose grabPose3 = new Pose(31.7, 15.1, Math.toRadians(-45));
    private final Pose hpPose = new Pose(24, 23.5, Math.toRadians(-150));
    private final Pose pickUpPose = new Pose(15.8, 36, Math.toRadians(180));
    private final Pose parkPose = new Pose(13, 20, Math.toRadians(-135));

    private final Point gSControl = new Point(22, 50, Point.CARTESIAN);
    private final Point sControl1 = new Point(20, 49, Point.CARTESIAN);
    private final Point sControl2 = new Point(36, 67, Point.CARTESIAN);

    private final Point pControl1 = new Point(28, 55, Point.CARTESIAN);

    // Path Commands
    Command preloadCommand = () -> follower.followPath(preload);
    Command pickUp1Command = () -> follower.followPath(pickUp1);
    Command pickUp2Command = () -> follower.followPath(pickUp2);
    Command pickUp3Command = () -> follower.followPath(pickUp3);
    Command hp1Command = () -> follower.followPath(hp1);
    Command hp2Command = () -> follower.followPath(hp2);
    Command hp3Command = () -> follower.followPath(hp3);
    Command s2Set = () -> follower.followPath(speci2Set, true);
    Command s3Set = () -> follower.followPath(speci3Set);
    Command s4Set = () -> follower.followPath(speci4Set);
    Command s5Set = () -> follower.followPath(speci5Set);
    Command s2Score = () -> follower.followPath(speci2Score);
    Command s3Score = () -> follower.followPath(speci3Score);
    Command s4Score = () -> follower.followPath(speci4Score);
    Command parkCommand = () -> follower.followPath(park);
    // Deposit Commands
    Command grabDeposit = () -> deposit.closeClaw();
    Command releaseSpecimen = () -> deposit.openClaw();
    Command lockSpecimen = () -> {
        deposit.specimenScorePos();
        deposit.openClaw();
    };
    // Intake Commands
    Command grabIntake = () -> diffy.closeClaw();
    Command releaseSample = () -> diffy.openClaw();
    Command diffyRetract = () -> diffy.foldIRetract();
    Command diffyTransfer = () -> diffy.diffyTransfer();
    Command diffyInterposed = () -> diffy.diffyInterposed();
    Command diffyShift = () -> diffy.speciShiftClaw();

    CommandSequence scorePreload = new CommandSequence()
            .addCommand(this::mSpecimenSet)
            .addCommand(preloadCommand)
            .build();
    CommandSequence grabSample1 = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.1)
            .addCommand(pickUp1Command)
            .addWaitCommand(.5)
            .addCommand(this::mSampleGrab)
            .addWaitCommand(.5)
            .addCommand(this::mSlideRest)
            .build();
    CommandSequence human1 = new CommandSequence()
            .addWaitCommand(.1)
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp1Command)
            .addCommand(diffyInterposed)
            .build();
    CommandSequence grabSample2 = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(this::mSampleGrab)
            .addCommand(pickUp2Command)
            .addWaitCommand(.1)
            .build();
    CommandSequence human2 = new CommandSequence()
            .addWaitCommand(.1)
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp2Command)
            .addCommand(diffyInterposed)
            .build();
    CommandSequence grabSample3 = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(this::mSampleGrab)
            .addCommand(pickUp3Command)
            .build();
    CommandSequence human3 = new CommandSequence()
            .addWaitCommand(.1)
            .addCommand(grabIntake)
            .addWaitCommand(.2)
            .addCommand(hp3Command)
            .build();
    CommandSequence speci2Grab = new CommandSequence()
            .addCommand(releaseSample)
            .addWaitCommand(.2)
            .addCommand(s2Set)
            .addCommand(this::mSpecimenPickup)
            .addWaitCommand(.2)
            .build();
    CommandSequence scoreSpeci2 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.3)
            .addCommand(diffyRetract)
            .addWaitCommand(.4)
            .addCommand(s2Score)
            .addCommand(grabDeposit)
            .addWaitCommand(.2)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence speci3Grab = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.1)
            .addCommand(s3Set)
            .addWaitCommand(.7)
            .addCommand(this::mSlideRest)
            .addWaitCommand(.2)
            .build();
    CommandSequence scoreSpeci3 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.3)
            .addCommand(diffyRetract)
            .addWaitCommand(.4)
            .addCommand(s3Score)
            .addCommand(grabDeposit)
            .addWaitCommand(.2)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence speci4Grab = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.1)
            .addCommand(s4Set)
            .addWaitCommand(.7)
            .addCommand(this::mSlideRest)
            .addWaitCommand(.2)
            .build();
    CommandSequence scoreSpeci4 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.3)
            .addCommand(diffyRetract)
            .addWaitCommand(.4)
            .addCommand(s4Score)
            .addCommand(grabDeposit)
            .addWaitCommand(.2)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence speci5Grab = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.1)
            .addCommand(s5Set)
            .addWaitCommand(.8)
            .addCommand(this::mSlideRest)
            .addWaitCommand(.2)
            .build();
    CommandSequence scoreSpeci5 = new CommandSequence()
            .addCommand(diffyShift)
            .addWaitCommand(.2)
            .addCommand(diffyTransfer)
            .addWaitCommand(.3)
            .addCommand(diffyRetract)
            .addWaitCommand(.2)
            .addCommand(this::speci5)
            .addWaitCommand(.2)
            .addCommand(grabDeposit)
            .addWaitCommand(.2)
            .addCommand(this::mSpecimenSet)
            .build();
    CommandSequence parkSequence = new CommandSequence()
            .addCommand(lockSpecimen)
            .addWaitCommand(.2)
            .addCommand(releaseSpecimen)
            .addWaitCommand(.2)
            .addCommand(parkCommand)
            .addWaitCommand(1)
            .addCommand(this::mSpecimenPickup)
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
            .addCommandSequence(speci5Grab)
            .addCommandSequence(scoreSpeci5)
            .addCommandSequence(parkSequence)
            .build();

    @Override
    public void init() {
        deposit.init(hardwareMap);
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
        preload.setPathEndTimeoutConstraint(1.3);

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
                                new Point(pickUpPose.getX(), pickUpPose.getY()-1)))
                .setLinearHeadingInterpolation(hpPose.getHeading(), pickUpPose.getHeading()).build();

        speci2Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose.getX(), pickUpPose.getY() + 1),
                                sControl1,
                                sControl2,
                                new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading()).build();

        speci3Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pControl1,
                                new Point(pickUpPose.getX() + 1, pickUpPose.getY() + 1)))
                .setConstantHeadingInterpolation(scorePose.getHeading()).build();

        speci3Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose.getX() + 1, pickUpPose.getY() + 1),
                                sControl1,
                                sControl2,
                                new Point(scorePose.getX(), scorePose.getY())))
                .setConstantHeadingInterpolation(scorePose.getHeading()).build();

        speci4Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pControl1,
                                new Point(pickUpPose.getX() + 1, pickUpPose.getY() + 2)))
                .setConstantHeadingInterpolation(scorePose.getHeading()).build();

        speci4Score = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(pickUpPose.getX() + 1, pickUpPose.getY() + 2),
                                sControl1,
                                sControl2,
                                new Point(scorePose.getX(), scorePose.getY() - 2)))
                .setConstantHeadingInterpolation(scorePose.getHeading()).build();

        speci5Set = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(scorePose.getX(), scorePose.getY() - 2),
                                pControl1,
                                new Point(pickUpPose.getX() + 1.2, pickUpPose.getY() + 1.8)))
                .setConstantHeadingInterpolation(pickUpPose.getHeading())
                .build();

        speci5Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(scorePose.getX(),scorePose.getY()+10)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        park = new Path(
                new BezierCurve(
                        new Point(scorePose),
                        new Point(parkPose)
                )
        );
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        slides.reset();
        deposit.transferPos();
        deposit.openClaw();
        diffy.diffyTransfer();
        diffy.openClaw();
        slides.restPos();
    }

    @Override
    public void init_loop() {
        if (gamepad1.cross) {
            deposit.closeClaw();
        }
        slides.update();
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
        if (slides.reset) slides.update();
        follower.update();
        runCommands.run(follower.isBusy() || (slides.getError() > 5) || !runCommands.getCurrentCommand().hasCompleted || !slides.reset);
    }

    @Override
    public void stop() {
        deposit.specimenScorePos();
        slides.lock();
        deposit.openClaw();
    }

    public void mSampleGrab() {
        diffy.extendNeutral();
        diffy.diffyDown();
        diffy.setDif_ROLL(-4.9);
        diffy.openClaw();
    }

    public void mSpecimenPickup() {
        diffy.diffySearch();
        diffy.setExtendinator(Intake.maxExtendy + (Intake.extendy_IN - Intake.maxExtendy) * .78);
        diffy.centerRoll();
        diffy.openClaw();
    }

    public void mSpecimenSet() {
        slides.setTarget(OuttakeSlides.HIGH_CHAMBER_SET - 60);
        deposit.closeClaw();
        deposit.specimenPos();
        mSpecimenPickup();
    }

    public void mSlideRest() {
        slides.setTarget(OuttakeSlides.INTAKE_POS);
        deposit.openClaw();
        deposit.transferPos();
    }

    public void speci5(){
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        speci5Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(scorePose.getX(),scorePose.getY()-5)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        follower.followPath(speci5Score);
    }
}