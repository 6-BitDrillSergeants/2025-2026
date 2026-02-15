package org.firstinspires.ftc.teamcode.subsystems.commands;

import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;

/**
 * Finishes once the supplied condition becomes true.
 */
public class WaitUntilCommand extends Command {

    private final BooleanSupplier condition;

    public WaitUntilCommand(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public boolean isDone() {
        return condition.getAsBoolean();
    }
}
