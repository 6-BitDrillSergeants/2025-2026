package org.firstinspires.ftc.teamcode.subsystems.config;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.feedback.PIDCoefficients;

/** Tunables for flywheel velocity control and conversion constants. */
@Configurable
public final class FlywheelConfig {
    private FlywheelConfig() {
    }

    public static double ticksPerRev = 28.0;
    public static double gearRatio = 1.0;

    // Target velocity in flywheel RPM.
    public static double targetRpm = 0.0;

    // Readiness threshold in RPM.
    public static double toleranceRpm = 50.0;

    // PID (NextControl).
    public static PIDCoefficients pid = new PIDCoefficients(0.011, 0.0, 0.0);

    // Feedforward (basicFF: v, a, s).
    public static double ffV = 0.00038; // power per (tick/sec)
    public static double ffA = 0.0;     // usually 0 unless profiling
    public static double ffS = 0.0;     // static friction
}
