package org.firstinspires.ftc.teamcode.opMode.teleop.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public enum FoozPadColors {
    NURAZ_DEFAULT(new Gamepad.LedEffect.Builder()
            .addStep(0.8 + (1-0.8) * 0/7, 0.5 + (1-0.5) * 0/7, 0.8 + (1-0.8) * 0/7, 200)
            .addStep(0.8 + (1-0.8) * 1/7, 0.5 + (1-0.5) * 1/7, 0.8 + (1-0.8) * 1/7, 100)
            .addStep(0.8 + (1-0.8) * 2/7, 0.5 + (1-0.5) * 2/7, 0.8 + (1-0.8) * 2/7, 100)
            .addStep(0.8 + (1-0.8) * 3/7, 0.5 + (1-0.5) * 3/7, 0.8 + (1-0.8) * 3/7, 100)
            .addStep(0.8 + (1-0.8) * 4/7, 0.5 + (1-0.5) * 4/7, 0.8 + (1-0.8) * 4/7, 100)
            .addStep(0.8 + (1-0.8) * 5/7, 0.5 + (1-0.5) * 5/7, 0.8 + (1-0.8) * 5/7, 100)
            .addStep(0.8 + (1-0.8) * 6/7, 0.5 + (1-0.5) * 6/7, 0.8 + (1-0.8) * 6/7, 100)
            .addStep(0.8 + (1-0.8) * 7/7, 0.5 + (1-0.5) * 7/7, 0.8 + (1-0.8) * 7/7, 200)
            .addStep(0.8 + (1-0.8) * 6/7, 0.5 + (1-0.5) * 6/7, 0.8 + (1-0.8) * 6/7, 100)
            .addStep(0.8 + (1-0.8) * 5/7, 0.5 + (1-0.5) * 5/7, 0.8 + (1-0.8) * 5/7, 100)
            .addStep(0.8 + (1-0.8) * 4/7, 0.5 + (1-0.5) * 4/7, 0.8 + (1-0.8) * 4/7, 100)
            .addStep(0.8 + (1-0.8) * 3/7, 0.5 + (1-0.5) * 3/7, 0.8 + (1-0.8) * 3/7, 100)
            .addStep(0.8 + (1-0.8) * 2/7, 0.5 + (1-0.5) * 2/7, 0.8 + (1-0.8) * 2/7, 100)
            .addStep(0.8 + (1-0.8) * 1/7, 0.5 + (1-0.5) * 1/7, 0.8 + (1-0.8) * 1/7, 100)
            .setRepeating(true)
            .build()
    ),
    SARAH_INTAKE(
            new Gamepad.LedEffect.Builder()
                    .addStep((double) 254 /255, (double) 69 /255, (double) 77 /255, 10000)
                    .setRepeating(true)
                    .build()
    ),
    SARAH_OUTTAKE(
            new Gamepad.LedEffect.Builder()
                    .addStep(0.8*.6, 0.52*.6, 0.8*.6, 10000)
                    .setRepeating(true)
                    .build()
    );

    public final Gamepad.LedEffect colorPattern;

    FoozPadColors(Gamepad.LedEffect pattern) {
        this.colorPattern = pattern;
    }
}
