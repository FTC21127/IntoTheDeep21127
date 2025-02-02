package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opMode.teleop.Utils.FoozPadUtils.FoozPadRumble.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fissionlib.command.Command;
import org.firstinspires.ftc.teamcode.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.fissionlib.input.FoozPad;
import org.firstinspires.ftc.teamcode.fissionlib.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opMode.auton.utils.Colors;

@Config
public class Intake extends Mechanism {

    // Roll is the turn the claw side to side
    // pitch is up and down

    ServoImplEx horizontalExtendenator;
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
    public static double c_CLOSE = 0.9;
    public static double c_SHIFT = 0.87;
    public static double extendy_IN = 0.7;
    public static double maxExtendy = 0.25;
    public static double extendy_NEUTRAL = (extendy_IN + maxExtendy)/2;
    public static double dif_TRANSFER = 0.76;
    public static double dif_INTERPOSED = .4;
    public static double dif_NEUTRAL = 0.6;
    public static double dif_DOWN = 0.1;
    public static double dif_SEARCH = 0.2;
    public static double dif_ROLL = 0;
    public double dif_PITCH = dif_TRANSFER;

    public boolean isPickup = false;
    public boolean isSearch = false;
    public boolean isExtended = false;

    CommandSequence retract = new CommandSequence()
            .addCommand(this::diffyDown)
            .addWaitCommand(.2)
            .addCommand(this::closeClaw)
            .addWaitCommand(.1)
            .addCommand(this::centerRoll)
            .addWaitCommand(.1)
            .addCommand(this::retractExtendy)
            .addCommand(this::shiftClaw)
            .addCommand(this::diffyTransfer)
            .build();

    public Intake(OpMode opMode1){
        this(opMode1,Colors.RED);
    }

    public Intake(OpMode opMode1, Colors alliance) {
        this.opMode = opMode1;
        this.alliance = alliance;
    }

    public void setAlliance(Colors alliance) {
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

    public void extendMax(){
        horizontalExtendenator.setPosition(extendy_NEUTRAL);
    }

    public void retractExtendy(){
        horizontalExtendenator.setPosition(extendy_IN);
    }

    public void setExtendinator(double pos){
            horizontalExtendenator.setPosition(Math.min(Math.max(maxExtendy,pos),extendy_IN));
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

    public void diffyInterposed(){
        dif_PITCH = dif_INTERPOSED;
        dif_ROLL = 0;
        intakeState = DiffyState.NEUTRAL;
    }

    public void diffyNeutral(){
        dif_PITCH = dif_NEUTRAL;
        dif_ROLL = 0;
        intakeState = DiffyState.NEUTRAL;
    }

    public void centerRoll(){
        setDif_ROLL(0);
        autoUpdate();
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
        horizontalExtendenator = hwMap.get(ServoImplEx.class, "horizontalExtendy");
        diffyLeft = hwMap.get(Servo.class, "diffyLeft");
        diffyRight = hwMap.get(Servo.class, "diffyRight");
        claw = hwMap.get(Servo.class, "intakeClaw");
        colorSensor = new IntakeSenor(opMode, alliance);
    }

    public void teleControl(FoozPad gp){
        dif_ROLL = (int)(gp.gamepad.right_stick_x*5)/50.0;
        setExtendinator(maxExtendy + Math.abs(gp.gamepad.left_stick_y)*(extendy_IN-maxExtendy)*.9);
    }

    public void autoUpdate(){
        diffyRight.setPosition(MathUtils.clamp(1 - dif_PITCH + dif_ROLL,0,1));
        diffyLeft.setPosition(MathUtils.clamp(dif_PITCH + dif_ROLL,0,1));
    }

    public void setUp(FoozPad gamepad){
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.A)) {
            claw.setPosition(c_OPEN);
        } else {
            claw.setPosition(c_CLOSE);
        }
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.DPAD_UP)){
            setExtendinator(maxExtendy);
        } else {
            setExtendinator(extendy_IN);
        }
    }

    @Override
    public void loop(FoozPad gamepad) {
        if (isPickup||isSearch) dif_ROLL = gamepad.gamepad.right_stick_x/10;
//        if (isExtended && gamepad.gamepad.left_stick_y != 0) setExtendinator(extendy_NEUTRAL + (gamepad.gamepad.left_stick_y*.9) * extendy_NEUTRAL);
        diffyRight.setPosition(MathUtils.clamp(1 - dif_PITCH + dif_ROLL,0,1));
        diffyLeft.setPosition(MathUtils.clamp(dif_PITCH + dif_ROLL,0,1));
        if (GamepadStatic.isButtonPressed(gamepad.gamepad, GamepadStatic.Input.RIGHT_BUMPER)){
//            if (colorSensor.isAllowed()){
                retract.trigger();
                isSearch = false;
                isPickup = false;
                isExtended = false;
//            } else {
//                gamepad.runRumbleEffect(INCORRECT_COLOR.rumblePattern);
//            }
        } else if (GamepadStatic.wasJustPressed(gamepad, GamepadStatic.Input.DPAD_LEFT)) {
            if (!isExtended){
                extendMax();
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

    public void turnOffExtendy(){
        horizontalExtendenator.setPwmDisable();
    }
}