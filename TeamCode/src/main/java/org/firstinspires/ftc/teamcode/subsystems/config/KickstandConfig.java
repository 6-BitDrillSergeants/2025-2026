package org.firstinspires.ftc.teamcode.subsystems.config;

import com.bylazar.configurables.annotations.Configurable;

/** Tunables for kickstand positions and motor power. */
@Configurable
public final class KickstandConfig {
    private KickstandConfig() {
    }

    public static int deployTicks = 800;
    public static int retractTicks = 0;
    public static double movePower = 0.5;
}
