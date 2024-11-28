package org.firstinspires.ftc.teamcode.opMode.teleop.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public enum FoozPadColors {
    NURAZ_DEFAULT(new Gamepad.LedEffect.Builder()
            .addStep(0.8 + (1-0.8) * 0/5, 0.5 + (1-0.5) * 0/5, 0.8 + (1-0.8) * 0/5, 250)
            .addStep(0.8 + (1-0.8) * 1/5, 0.5 + (1-0.5) * 1/5, 0.8 + (1-0.8) * 1/5, 250)
            .addStep(0.8 + (1-0.8) * 2/5, 0.5 + (1-0.5) * 2/5, 0.8 + (1-0.8) * 2/5, 250)
            .addStep(0.8 + (1-0.8) * 3/5, 0.5 + (1-0.5) * 3/5, 0.8 + (1-0.8) * 3/5, 250)
            .addStep(0.8 + (1-0.8) * 4/5, 0.5 + (1-0.5) * 4/5, 0.8 + (1-0.8) * 4/5, 250)
            .addStep(0.8 + (1-0.8) * 5/5, 0.5 + (1-0.5) * 5/5, 0.8 + (1-0.8) * 5/5, 250)
            .addStep(0.8 + (1-0.8) * 4/5, 0.5 + (1-0.5) * 4/5, 0.8 + (1-0.8) * 4/5, 250)
            .addStep(0.8 + (1-0.8) * 3/5, 0.5 + (1-0.5) * 3/5, 0.8 + (1-0.8) * 3/5, 250)
            .addStep(0.8 + (1-0.8) * 2/5, 0.5 + (1-0.5) * 2/5, 0.8 + (1-0.8) * 2/5, 250)
            .addStep(0.8 + (1-0.8) * 1/5, 0.5 + (1-0.5) * 1/5, 0.8 + (1-0.8) * 1/5, 250)
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
