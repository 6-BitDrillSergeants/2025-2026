package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.config.KickstandConfig;
import org.firstinspires.ftc.teamcode.subsystems.config.PaddleConfig;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/** Controls the kickstand motor position using RUN_TO_POSITION mode. */
public final class Kickstand implements Subsystem {
    private final MotorEx kickstandMotor = new MotorEx(RobotConfig.kickstandMotorName);

    /** Moves the kickstand to the configured deployed position. */
    public Command deploy() {
        return new SequentialGroup(new InstantCommand(() -> moveTo(-800)),
                new Delay(1),
                new InstantCommand(() -> moveTo(-1150)),
                new Delay(1),
                new InstantCommand(() -> moveTo(-1200)),
                new Delay(2),
                new InstantCommand(() -> moveTo(-1250)),
                new Delay(1)).requires(this);
    }

    /** Moves the kickstand to the configured retracted position. */
    public void retract() {
        moveTo(KickstandConfig.retractTicks);
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
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // No periodic logic needed.
    }
}
