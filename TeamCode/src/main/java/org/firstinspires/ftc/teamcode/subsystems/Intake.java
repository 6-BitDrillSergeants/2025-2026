package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.config.IntakeConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/** Controls the intake motor in two states: on (collecting) or off. */
public final class Intake implements Subsystem {
    private final MotorEx intakeMotor = new MotorEx(RobotConfig.intakeMotorName);

    /** Runs the intake at the configured collection power. */
    public void on() {
        intakeMotor.setPower(IntakeConfig.onPower);
    }

    /** Stops the intake motor. */
    public void off() {
        intakeMotor.setPower(IntakeConfig.offPower);
    }

    public boolean isOn() {
        return intakeMotor.getPower() > 0.0;
    }

    @Override
    public void periodic() {
        // No periodic logic needed.
    }
}
