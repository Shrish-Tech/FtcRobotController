package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleopRED extends OpMode {

    private Follower follower;

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretMechanism turret = new TurretMechanism();

    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private DcMotor intake;
    private DcMotor outtake;
    private Servo flap;
    private Servo hood;

    private Timer actionTimer;
    private boolean shootMode = false;
    

    public void intake(double speed) {
        intake.setPower(speed);
    }

    public void outtakeMode(String mode) {
        if (mode.equalsIgnoreCase("on")) {
            outtake.setPower(.7);
            setFlap(.4);
        } else {
            outtake.setPower(0);
            setFlap(.7);
        }
    }

    public void shoot(double speed) {
        intake.setPower(speed);
    }

    public void setFlap(double pos) {
        flap.setPosition(pos);
    }

    public void set_calc_hood_angle(double ftcPose){
        //calculation
        //hood.setPosition(x)
    }

    public void set_hood(double distance) {
        hood.setPosition(distance);
    }


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        flap = hardwareMap.get(Servo.class, "flap");
        hood = hardwareMap.get(Servo.class, "hood");

        telemetry.addLine("All INITED");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.restartTimer();
    }


    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTageBySpecificId(24);

        if (id24 != null) {

            telemetry.addData("Tag Detected", id24.id);

            // turret tracking
            turret.update(id24);

            // hood angle calculation
            double distanceY = id24.ftcPose.y;
            set_calc_hood_angle(distanceY);

            // shooter logic
            if (distanceY < 65) {
                outtakeMode("on");
                shootMode = true;
            } else {
                outtakeMode("off");
                shootMode = false;
            }

        } else {
            telemetry.addLine("NO TAG DETECTED");
            outtakeMode("off");
            shootMode = false;
        }

        if (gamepad2.right_trigger > 0.1 && shootMode) {
            shoot(1);
        } else {
            shoot(0);
        }

        if (gamepad2.left_trigger > 0.1) {
            intake(1);
        } else {
            intake(0);
        }
        if(gamepad2.dpad_up){
            set_hood(1);
        }
        if(gamepad2.dpad_down){
            set_hood(.75);
        }
        if(gamepad2.dpad_right){
            set_hood(.5);
        }
        if(gamepad2.dpad_left   ){
            set_hood(.25);
        }
        if(gamepad2.b){
            flap.setPosition(0.4);
        }
        if(gamepad2.a){
            flap.setPosition(0.7);
        }

        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            }
        }
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}