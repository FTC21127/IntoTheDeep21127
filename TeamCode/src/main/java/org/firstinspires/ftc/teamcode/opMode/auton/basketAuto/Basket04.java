package org.firstinspires.ftc.teamcode.opMode.auton.basketAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlides;

public class Basket04 extends OpMode {
    Deposit deposit = new Deposit(this);
    Drivetrain drivetrain = new Drivetrain(this);
    OuttakeSlides slides = new OuttakeSlides(this);
    Intake diffy = new Intake(this);

    Follower follower;

    private final Pose startPose = new Pose(7, 103, Math.toRadians(270));
    private final Pose scorePose = new Pose(39, 66, Math.toRadians(315));
    private final Pose grab1Pose = new Pose(39, 120, Math.toRadians(0));
    private final Pose grab2Pose = new Pose(39, 120, Math.toRadians(0));
    private final Pose grab3Pose = new Pose(39, 120, Math.toRadians(90));
    private final Pose parkPose = new Pose(26, 30, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup;


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


    }

    @Override
    public void loop() {

    }
}
