package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.config.PaddleConfig;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

/** Controls the feeder paddle servo used to advance game pieces into the shooter. */
public final class Paddle implements Subsystem {

    public static final Paddle INSTANCE = new Paddle();

    private final ServoEx paddleServo = new ServoEx("paddle");

    private Paddle() {
    }

    /** Simple command to move the paddle down. */
    public final Command lower = new SetPosition(paddleServo, PaddleConfig.downPosition).requires(this);

    /** Simple command to move the paddle up. */
    public final Command raise = new SetPosition(paddleServo, PaddleConfig.upPosition).requires(this);

    /**
     * Runs one feed cycle: raise -> wait -> lower.
     */
    public Command feedOnce() {
        return new SequentialGroup(raise, new Delay(PaddleConfig.feedTimeSeconds), lower).requires(this);
    }

    @Override
    public void periodic() {
        // No periodic updates for servos.
    }
}
