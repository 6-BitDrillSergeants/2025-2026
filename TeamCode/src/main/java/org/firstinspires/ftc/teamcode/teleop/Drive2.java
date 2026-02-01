package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelController;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelTuning;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Drive2 extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean kickvar = false; //
    private boolean automatedDrive;
    private TelemetryManager telemetryM;

    AimingCalculator aimCalc;

    DcMotorEx intakeMotor;

    DcMotorEx flywheelMotor;
    Servo paddleServo;
    LauncherFlywheelController controller;

    DcMotorEx kickstandMotor;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.update();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        paddleServo = hardwareMap.get(Servo.class, "paddleServo");
        paddleServo.setPosition(.58); // .35

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new LauncherFlywheelController(flywheelMotor);

        //kickstandMotor = hardwareMap.get(DcMotorEx.class, "kickstandMotor");
        //kickstandMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //kickstandMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        LauncherFlywheelTuning.targetVelocity = 1500;
    }

    @Override
    public void loop() {
        controller.update();
        follower.update();
        telemetry.update();
        //telemetryM.update();

        if (!automatedDrive) {
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }


        if (gamepad1.rightBumperWasPressed()) {
           /* paddleServo.setPosition(.83); // .60
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            paddleServo.setPosition(.58); // .35*/

            Pose target = AimingCalculator.computeAimPose(follower.getPose(), AimingCalculator.Goal.BLUE_GOAL);
            follower.holdPoint(target);
        }



        if (gamepad1.aWasPressed()){
            intakeMotor.setPower(1);
        } else if (gamepad1.bWasPressed()){
            intakeMotor.setPower(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            LauncherFlywheelTuning.targetVelocity += 20;
        } else if (gamepad1.dpadDownWasPressed()) {
            LauncherFlywheelTuning.targetVelocity -= 20;
        }

        if (gamepad1.xWasPressed()){
            kickstandMotor.setTargetPosition(800);
            kickstandMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            kickvar = true;
            kickstandMotor.setPower(1);
        }

        telemetry.addData("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);

//        telemetryM.debug("targetVelocity", LauncherFlywheelTuning.targetVelocity);
//        telemetryM.debug("actualVelocity", flywheelMotor.getVelocity());

  /*      telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
*/
        telemetry.addData("targetVelocity", LauncherFlywheelTuning.targetVelocity);
        telemetry.addData("actualVelocity", flywheelMotor.getVelocity());
        //telemetry.addData("kickstandPosition", kickstandMotor.getCurrentPosition());
        //telemetry.addData("kickstand button was pressed", kickstandMotor.getCurrent(CurrentUnit.AMPS));

        //telemetry.addData("kickstand current", hardwareMap.get(DcMotorEx.class, "kickstandMotor").getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("intake current", hardwareMap.get(DcMotorEx.class, "intakeMotor").getCurrent(CurrentUnit.AMPS));
        telemetry.addData("launcher current", flywheelMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("launcher power", flywheelMotor.getPower());
        //telemetry.addData("rf current", hardwareMap.get(DcMotorEx.class, "rightFront").getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("lf current", hardwareMap.get(DcMotorEx.class, "leftFront").getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("rb current", hardwareMap.get(DcMotorEx.class, "rightBack").getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("lb current", hardwareMap.get(DcMotorEx.class, "leftBack").getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }
}