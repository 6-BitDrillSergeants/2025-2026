package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private LauncherController launcher;
    private PaddleController paddle;

    DcMotorEx motor1;// = hardwareMap.get(DcMotorEx .class, "motor1");
    DcMotorEx motor2;// = hardwareMap.get(DcMotorEx.class, "motor2");

    Servo light;

    private int loopCount = 0;

    private boolean reverse = true;

    public Follower follower;

    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        paddle = new PaddleController(hardwareMap);
        launcher = new LauncherController(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        light = hardwareMap.get(Servo.class, "light");

        follower = Constants.createFollower(hardwareMap);
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        follower.activateHeading();
        //follower.activateDrive();
    }

    @Override
    public void stop() {

    }

    @Override
    public void start() {

    }

    private boolean isHolding = false;
    private boolean hasMoved = false;
    private Pose holdPosition;

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        // Inverting controller inputs for cup 1

        if (gamepad1.left_stick_x == 0 &&
                gamepad1.left_stick_y == 0 &&
                gamepad1.right_stick_x == 0 &&
                gamepad1.right_stick_y == 0
        ) {
            if (!isHolding && hasMoved) {
                follower.holdPoint(follower.getPose());
                isHolding = true;
            }
        } else {
            isHolding = false;
            hasMoved = true;

            light.setPosition(.3);//red
            driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }


        if (gamepad1.aWasPressed()) {
            launcher.runFast();
        } else if (gamepad1.xWasPressed()) {
            launcher.runSlow();
        } else if (gamepad1.bWasPressed()) {
            launcher.stop();
        } else if (gamepad1.yWasPressed()) {
            paddle.raisePaddle();
            loopCount = 0;
        }

        if (gamepad1.rightBumperWasPressed()) {
            reverse = !reverse;
        }

        if (paddle.isRaised()) {
            loopCount++;
            if (loopCount >= 25) {
                paddle.lowerPaddle();
            }
        }

        telemetry.addData("Raised:", paddle.isRaised());
        telemetry.addData("Motor 1 Power:", motor1.getPower());
        telemetry.addData("Motor 2 Power:", motor2.getPower());
        telemetry.addData("Motor 1 Speed:", motor1.getVelocity());
        telemetry.addData("Motor 2 Speed:", motor2.getVelocity());
        telemetry.addData("Pose", follower.getPose());

        telemetry.update();
        follower.update();
    }
}