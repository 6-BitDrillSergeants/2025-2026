package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.config.IntakeConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/** Controls the intake motor in two states: on (collecting) or off. */
public final class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    private final MotorEx intakeMotor = new MotorEx("intake");

    private Intake() {
    }

    /** Runs the intake at the configured collection power. */
    public void on() {
        intakeMotor.setPower(IntakeConfig.onPower);
    }

    /** Stops the intake motor. */
    public void off() {
        intakeMotor.setPower(IntakeConfig.offPower);
    }

    @Override
    public void periodic() {
        // No periodic logic needed.
    }
}
