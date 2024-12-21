package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.FoozPadRumble.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;
import org.firstinspires.ftc.teamcode.opMode.teleop.ControlsM3;

@Config
public class Intake extends Mechanism {

    // Roll is the turn the claw side to side
    // pitch is up and down

    Servo horizontalExtendenator;
    Servo diffyLeft, diffyRight;
    Servo claw;

    IntakeSenor colorSensor;

    Colors alliance;

    public enum DiffyState{
        DOWN,
        TRANSFER,
        SEARCH,
        NEUTRAL,
    }
    DiffyState intakeState = DiffyState.NEUTRAL;

    public static double c_OPEN = 0.5;
    public static double c_CLOSE = 0.6;
    public static double c_SHIFT = 0.55;
    public static double extendy_NEUTRAL = 0.5;
    public static double extendy_IN = 0;
    public static double maxExtendy = 0.7;
    public static double dif_TRANSFER = 1;
    public static double dif_NEUTRAL = .9;
    public static double dif_DOWN = 0.375;
    public static double dif_SEARCH = 0.45;
    public static double dif_ROLL = 0;
    public double dif_PITCH = dif_NEUTRAL;

    public boolean isPickup = false;
    public boolean isSearch = false;
    public boolean isExtended = false;

    public Intake(OpMode opMode1){
        this(opMode1,Colors.RED);
    }

    public Intake(OpMode opMode1, Colors alliance) {
        this.opMode = opMode1;
        this.alliance = alliance;
    }

    public DiffyState getIntakeState() {
        return intakeState;
    }

    public void setDif_ROLL(double degrees) {
        degrees /= 90;
        Intake.dif_ROLL = Math.min(Math.max(-.1,degrees),.1);
    }

    public void extendNeutral(){
        horizontalExtendenator.setPosition(extendy_NEUTRAL);
    }

    public void retractExtendy(){
        horizontalExtendenator.setPosition(extendy_IN);
    }

    public void setExtendinator(double pos){
            horizontalExtendenator.setPosition(Math.min(maxExtendy,pos));
    }

    public void diffyDown(){
        dif_PITCH = dif_DOWN;
        intakeState = DiffyState.DOWN;
    }

    public void diffySearch(){
        dif_PITCH = dif_SEARCH;
        intakeState = DiffyState.SEARCH;
    }

    public void diffyTransfer(){
        dif_PITCH = dif_TRANSFER;
        dif_ROLL = 0;
        intakeState = DiffyState.TRANSFER;
    }

    public void diffyNeutral(){
        dif_PITCH = dif_NEUTRAL;
        intakeState = DiffyState.NEUTRAL;
    }

    public void closeClaw(){
        claw.setPosition(c_CLOSE);
    }
    public void openClaw(){
        claw.setPosition(c_OPEN);
    }
    public void shiftClaw(){
        claw.setPosition(c_SHIFT);
    }

    @Override
    public void init(HardwareMap hwMap) {
        horizontalExtendenator = hwMap.get(Servo.class, "horizontalExtendy");
        diffyLeft = hwMap.get(Servo.class, "diffyLeft");
        diffyRight = hwMap.get(Servo.class, "diffyRight");
        claw = hwMap.get(Servo.class, "intakeClaw");
        colorSensor = new IntakeSenor(opMode, alliance);
    }

    public void autoUpdate(){
        diffyRight.setPosition(1 - dif_PITCH + dif_ROLL);
        diffyLeft.setPosition(dif_PITCH + dif_ROLL);
    }

    @Override
    public void loop(FoozPad gamepad) {
        if (isPickup||isSearch) dif_ROLL = gamepad.gamepad.right_stick_x/10;
        if (isExtended && gamepad.gamepad.left_stick_y != 0) setExtendinator(extendy_NEUTRAL + gamepad.gamepad.left_stick_y/2.2);
        diffyRight.setPosition(1 - dif_PITCH + dif_ROLL/10);
        diffyLeft.setPosition(dif_PITCH + dif_ROLL/10);
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, ControlsM3.GRAB)){
            if (colorSensor.isAllowed()){
                closeClaw();
                diffyNeutral();
                retractExtendy();
                isSearch = false;
                isPickup = false;
                isExtended = false;
            } else {
                gamepad.runRumbleEffect(INCORRECT_COLOR.rumblePattern);
            }
        } else if (GamepadStatic.wasJustPressed(gamepad, ControlsM3.SEARCH)) {
            if (!isExtended){
                extendNeutral();
                isExtended = true;
            }
            if (isSearch) {
                diffyDown();
                isSearch = false;
                isPickup = true;
            } else {
                diffySearch();
                isSearch = true;
                isPickup = false;
            }
            openClaw();
        }
    }

    @Config
    class IntakeSenor extends Mechanism {
        RevColorSensorV3 colorSensor;
        Colors alliance;

        double SAMPLE_DISTANCE_CM = 4.2;

        public IntakeSenor(OpMode opMode1, Colors alliance) {
            this.opMode = opMode1;
            this.alliance = alliance;
        }

        @Override
        public void init(HardwareMap hwMap) {
            colorSensor = hwMap.get(RevColorSensorV3.class, "color");
            colorSensor.setGain(35);
        }

        public double getDistance(){
           return colorSensor.getDistance(DistanceUnit.CM);
        }

        public boolean isClose(){
            return getDistance() <= SAMPLE_DISTANCE_CM;
        }

        public double[] getNormRGB(){
            return new double[]{colorSensor.getNormalizedColors().red,colorSensor.getNormalizedColors().green,colorSensor.getNormalizedColors().blue};
        }

        public boolean isAllianceSpecific(){
            return Colors.isColor(getNormRGB()[0],getNormRGB()[1],getNormRGB()[2]) == alliance;
        }

        public boolean isAllowed(){
            return Colors.isColor(getNormRGB()[0],getNormRGB()[1],getNormRGB()[2]) == alliance || Colors.isColor(getNormRGB()[0],getNormRGB()[1],getNormRGB()[2]) == Colors.YELLOW;
        }
    }
}