package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private LauncherController launcher;
    private PaddleController paddle;

    private int loopCount = 0;
   
    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        paddle = new PaddleController(hardwareMap);
        launcher = new Launcher(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void stop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        // multiply left stick y by -1 to invert direction (forward on the joystick gives negative values)
        //driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);
        
        // Inverting controller inputs for cup 1
        driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x* -1);

        if (gamepad1.aWasPressed()) {
            launcher.runFast();
        } 
        else if (gamepad1.xWasPressed()) {
            launcher.runSlow();
        } 
        else if (gamepad1.bWasPressed()) {
            launcher.stop();
        }
        else if (gamepad1.yWasPressed()) {
            paddle.raisePaddle();
            loopCount = 0;
            raised = true;
        }

        if (paddle.isRaised()) {
            loopCount++;
            if (loopCount >= loopThreshold ) {
                paddle.lowerPaddle();
                raised =false;
            }
        }
        
        telemetry.update();
    }
}