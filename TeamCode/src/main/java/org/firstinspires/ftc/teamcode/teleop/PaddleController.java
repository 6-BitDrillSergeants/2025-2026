package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PaddleController {
    private Servo servo;
    private final double raisedPosition = 0.55;
    private final double loweredPosition = 0.5;
    private boolean isRaised = false;

    public DriveController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "paddleServo");
    }

    public void raisePaddle() {
        servo.setPosition(raisedPosition);
        isRaised = true;
    }

    public void lowerPaddle() {
        servo.setPosition(loweredPosition);
        isRaised = false;
    }

    public boolean isRaised() {
        return isRaised;
    }
}
