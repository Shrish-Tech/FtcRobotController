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

    private AprilTagWebcam  aprilTagWebcam = new AprilTagWebcam();
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
    private Timer actionTimer;
    private boolean shootMode = false;


    public void intake(double speed){
        setFlap(.5);
        intake.setPower(speed);
    }
    public void outtakeMode(String mode){
        if(mode.equalsIgnoreCase("on")) {
            outtake.setPower(1);
            setFlap(0);
        }
        else {
            outtake.setPower(0);
            setFlap(.5);
        }
    }
    public void shoot(double speed){
        intake.setPower(speed);
    }
    public void setFlap(double pos){
        flap.setPosition(pos);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        aprilTagWebcam.init(hardwareMap,telemetry);
        turret.init(hardwareMap);
        intake = hardwareMap.get(DcMotor.class,"intake");
        outtake = hardwareMap.get(DcMotor.class,"outtake");
        flap = hardwareMap.get(Servo.class,"flap");

        telemetry.addLine("All INITED");

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.restartTimer();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        //webcam starting phase
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTageBySpecificId(24);
        //starting turret
        turret.update(id20);
        //telemetery feedback
        if(id20 != null){
            telemetry.addData("Cur Id", aprilTagWebcam);
        }else{
            telemetry.addLine("NO TAG DETECTED");
        }
        if(id20.ftcPose.y<50){
            outtakeMode("on");
            shootMode = true;
        }
        else{
            outtakeMode("off");
            shootMode = false;
        }
        if(gamepad2.right_trigger>0.1 && shootMode ){
            shoot(1);
        }
        else{
            shoot(0);
        }
        if(gamepad2.left_trigger>0.1){
            intake(1);
        }
        else{
            intake(0);
        }

        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }

//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

//        Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}