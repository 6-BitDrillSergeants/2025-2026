package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public final class Paddle implements Subsystem {

    public static final Paddle INSTANCE = new Paddle();

    private Paddle() {
    }

    private final ServoEx paddleServo = new ServoEx("paddleServo");

    private static final double DOWN = 0.58;

    private static final double UP = 0.83;

    private static final double FEED_TIME_SEC = 0.25;

    // Simple commands you can reuse elsewhere
    public final Command lower = new SetPosition(paddleServo, DOWN).requires(this);

    public final Command raise = new SetPosition(paddleServo, UP).requires(this);

    /**
     * One feed cycle: raise -> wait -> lower.
     */
    public Command feedOnce() {
        return new SequentialGroup(raise, new Delay(FEED_TIME_SEC),
                lower).requires(this);
    }

    @Override
    public void periodic() {
        // no periodic updates for servos.
    }
}

