package org.firstinspires.ftc.teamcode.subsystems.config;

import com.bylazar.configurables.annotations.Configurable;

/** Tunables for the intake subsystem. */
@Configurable
public final class IntakeConfig {
    private IntakeConfig() {
    }

    public static double onPower = 1.0;
    public static double offPower = 0.0;
}
