package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystems {

    private Follower follower;
    private DcMotor intake;
    private DcMotor outtake;
    private Servo flap;
    private Servo hood;

    public void init(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        flap = hardwareMap.get(Servo.class, "flap");
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public Follower getFollower() {
        return follower;
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void updateFollower() {
        follower.update();
    }

    public void setDrive(double y, double x, double turn) {
        follower.setTeleOpDrive(y, x, turn, true);
    }

    public void intake(double speed) {
        intake.setPower(speed);
    }

    public void shoot(double speed) {
        intake.setPower(speed);
    }

    public void outtakeOn() {
        outtake.setPower(0.7);
        flap.setPosition(0.4);
    }

    public void outtakeOff() {
        outtake.setPower(0);
        flap.setPosition(0.7);
    }

    public void setFlap(double pos) {
        flap.setPosition(pos);
    }

    public void setHood(double pos) {
        hood.setPosition(pos);
    }

    public void autoHoodFromDistance(double distance) {
        double pos = distance / 100.0;
        pos = Math.max(0.0, Math.min(1.0, pos));
        hood.setPosition(pos);
    }
}