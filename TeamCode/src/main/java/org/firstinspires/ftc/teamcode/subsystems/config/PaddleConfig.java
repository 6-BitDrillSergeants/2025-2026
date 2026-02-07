package org.firstinspires.ftc.teamcode.subsystems.config;

import com.bylazar.configurables.annotations.Configurable;

/** Tunables for the paddle subsystem. */
@Configurable
public final class PaddleConfig {
    private PaddleConfig() {
    }

    public static double downPosition = 0.58;
    public static double upPosition = 0.83;
    public static double feedTimeSeconds = 0.25;
}
