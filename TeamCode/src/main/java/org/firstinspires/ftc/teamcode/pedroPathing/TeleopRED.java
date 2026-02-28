package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "Teleop RED")
public class TeleopRED extends OpMode {

    private Subsystems subsystems = new Subsystems();
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretMechanism turret = new TurretMechanism();

    public static Pose startingPose;
    private TelemetryManager telemetryM;
    private boolean shootMode = false;

    @Override
    public void init() {
        subsystems.init(hardwareMap, startingPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);
    }

    @Override
    public void start() {
        subsystems.startTeleopDrive();
        turret.restartTimer();
    }

    @Override
    public void loop() {
        subsystems.updateFollower();
        telemetryM.update();

        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTageBySpecificId(24);

        if (tag != null && tag.ftcPose != null) {
            turret.update(tag);
            subsystems.autoHoodFromDistance(tag.ftcPose.y);

            if (tag.ftcPose.y < 65) {
                subsystems.outtakeOn();
                shootMode = true;
            } else {
                subsystems.outtakeOff();
                shootMode = false;
            }
        } else {
            subsystems.outtakeOff();
            shootMode = false;
        }

        if (gamepad2.right_trigger > 0.1 && shootMode) {
            subsystems.shoot(1);
        } else {
            subsystems.shoot(0);
        }

        if (gamepad2.left_trigger > 0.1) {
            subsystems.intake(1);
        } else {
            subsystems.intake(0);
        }

        if (gamepad2.dpad_up) subsystems.setHood(1);
        if (gamepad2.dpad_down) subsystems.setHood(0.75);
        if (gamepad2.dpad_right) subsystems.setHood(0.5);
        if (gamepad2.dpad_left) subsystems.setHood(0.25);

        if (gamepad2.a) subsystems.setFlap(0.7);
        if (gamepad2.b) subsystems.setFlap(0.4);

        subsystems.setDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        telemetryM.debug("pose", subsystems.getFollower().getPose());
        telemetryM.debug("velocity", subsystems.getFollower().getVelocity());
    }
}