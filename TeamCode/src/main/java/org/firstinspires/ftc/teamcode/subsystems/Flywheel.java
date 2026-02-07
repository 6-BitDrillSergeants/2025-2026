package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static dev.nextftc.control.builder.ControlSystemBuilderKt.controlSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import kotlin.Unit;

/**
 * Controls shooter flywheel speed using velocity PID + feedforward.
 *
 * <p>Supports either manual RPM targets or automatic distance-based targets.
 */
public final class Flywheel implements Subsystem {

    public static final Flywheel INSTANCE = new Flywheel();

    private Flywheel() {
    }

    private final MotorEx flywheelMotor = new MotorEx("flywheel");

    private ControlSystem controller;

    // Runtime state
    private boolean autoFromDistance = false;
    private double targetRpm = 0.0;

    /**
     * Enable distance-based target RPM.
     */
    public void enableAutoFromDistance() {
        autoFromDistance = true;
    }

    /**
     * Disable distance-based target RPM.
     */
    public void disableAutoFromDistance() {
        autoFromDistance = false;
    }

    /**
     * Manual RPM target. Disables auto mode.
     */
    public void setTargetRpm(double rpm) {
        autoFromDistance = false;
        targetRpm = max(0.0, rpm);
        FlywheelConfig.targetRpm = targetRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        DcMotorEx motor = flywheelMotor.getMotor();
        double motorRevPerSec = motor.getVelocity() / FlywheelConfig.ticksPerRev;
        double flywheelRevPerSec = motorRevPerSec * FlywheelConfig.gearRatio;
        return flywheelRevPerSec * 60.0;
    }

    public boolean isAtSpeed() {
        return abs(FlywheelConfig.targetRpm - ticksPerSecondToRpm(flywheelMotor.getVelocity())) <= FlywheelConfig.toleranceRpm;
    }

    public void stop() {
        autoFromDistance = false;
        targetRpm = 0.0;
        FlywheelConfig.targetRpm = 0.0;
    }

    @Override
    public void periodic() {
        // Rebuild the controller in case any config variables changed
        rebuildController();

        if (autoFromDistance) {
            double distance = DistanceProvider.INSTANCE.getDistance();
            targetRpm = rpmForDistance(distance);
            FlywheelConfig.targetRpm = targetRpm;
        }

        controller.setGoal(new KineticState(0.0, rpmToTicksPerSecond(FlywheelConfig.targetRpm)));
        double power = controller.calculate(new KineticState(flywheelMotor.getCurrentPosition(), flywheelMotor.getVelocity()));

        flywheelMotor.setPower(power);
    }

    // ----------------------------
    // Distance -> RPM mapping
    // ----------------------------

    /**
     * Quadratic fit:
     * y = 0.3666 x^2 - 6.59 x + 500
     * clamped to [0, 80] distance range.
     */
    private double rpmForDistance(double distanceRaw) {
        double d = clamp(distanceRaw, 0.0, 80.0);
        double rpm = 0.3666 * d * d - 6.59 * d + 500.0;
        return max(0.0, rpm);
    }

    // ----------------------------
    // Units
    // ----------------------------

    private double rpmToTicksPerSecond(double flywheelRpm) {
        double flywheelRevPerSec = flywheelRpm / 60.0;
        double motorRevPerSec = (FlywheelConfig.gearRatio == 0.0) ? 0.0 : (flywheelRevPerSec / FlywheelConfig.gearRatio);
        return motorRevPerSec * FlywheelConfig.ticksPerRev;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        double motorRevPerSec = (FlywheelConfig.ticksPerRev == 0.0) ? 0.0 : (ticksPerSecond / FlywheelConfig.ticksPerRev);
        double flywheelRevPerSec = motorRevPerSec * FlywheelConfig.gearRatio;
        return flywheelRevPerSec * 60.0;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void rebuildController() {
        controller = controlSystem(system -> {
            system.velPid(FlywheelConfig.pid);
            system.basicFF(FlywheelConfig.ffV, FlywheelConfig.ffA, FlywheelConfig.ffS);
            return Unit.INSTANCE;
        });
    }
}
