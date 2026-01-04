package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.ServoEx;

// --- Subsystem: Launcher (flywheel + feeder) ---
public class LauncherOld implements Subsystem {

    public static final LauncherOld INSTANCE = new LauncherOld();

    // Feeder positions
    private static final double FEED_POS = 0.57;
    private static final double REST_POS = 0.5;

    ////private final MotorEx motor1 = new MotorEx("motor1");
   // private final MotorEx motor2 = new MotorEx("motor2");
    private final ServoEx paddle = new ServoEx("paddleServo");

    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.1)
            .build();

    public Command turnMotorsOff = new RunToPosition(controlSystem, 0).requires(this);
    public Command feedOne = new SequentialGroup(
            new InstantCommand(() -> paddle.setPosition(FEED_POS)).requires(this),
            new Delay(1), // seconds
            new InstantCommand(() -> paddle.setPosition(REST_POS)).requires(this)
    );

    @Override
    public void initialize() {
        paddle.setPosition(REST_POS);
    }

    @Override
    public void periodic() {
        ///motor1.setPower(controlSystem.calculate(motor1.getState()));
        //motor2.setPower(controlSystem.calculate(motor2.getState()));
    }

    public static class PanelsGraphTest {
    }
}
