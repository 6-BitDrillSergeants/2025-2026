package org.firstinspires.ftc.teamcode.pidTuning;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
public final class LauncherFlywheelTuning {

    private LauncherFlywheelTuning() {}

    // Target velocity in encoder ticks/sec
    public static double targetVelocity = 0.0;

    // PID (NextControl)
    public static PIDCoefficients pid =
            new PIDCoefficients(0.001, 0.0, 0.0);

    // Feedforward (basicFF: v, a, s)
    public static double ffV = 0.0005; // power per (tick/sec)
    public static double ffA = 0.0;    // usually 0 unless profiling
    public static double ffS = 0.0;    // static friction

    // Readiness threshold
    public static double toleranceVelocity = 50.0;
}
