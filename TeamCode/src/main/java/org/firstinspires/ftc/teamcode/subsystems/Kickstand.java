package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.config.KickstandConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/** Controls the kickstand motor position using RUN_TO_POSITION mode. */
public final class Kickstand implements Subsystem {

    public static final Kickstand INSTANCE = new Kickstand();

    private final MotorEx kickstandMotor = new MotorEx("kickstand");

    private Kickstand() {
    }

    /** Moves the kickstand to the configured deployed position. */
    public void deploy() {
        moveTo(KickstandConfig.deployTicks);
    }

    /** Moves the kickstand to the configured retracted position. */
    public void retract() {
        moveTo(KickstandConfig.retractTicks);
    }

    /** Returns whether the motor controller is still driving toward its target. */
    public boolean isBusy() {
        return kickstandMotor.getMotor().isBusy();
    }

    private void moveTo(int ticks) {
        DcMotorEx motor = kickstandMotor.getMotor();

        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(KickstandConfig.movePower);
    }

    @Override
    public void initialize() {
        Subsystem.super.initialize();

        DcMotorEx motor = kickstandMotor.getMotor();
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // No periodic logic needed.
    }
}
