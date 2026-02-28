package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretMechanism {

    private DcMotorEx turret;

    private double kP = 0.01;
    private double kD = 0.001;

    private double goalX = 0;
    private double lastError = 0;
    private double angleTol = 0.2;
    private final double MAX_POWER = 0.8;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    public void restartTimer() {
        timer.reset();
    }

    public void setGoal(double goal) {
        goalX = goal;
    }

    public void update(AprilTagDetection tag) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (tag == null || tag.ftcPose == null) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - tag.ftcPose.bearing;

        double p = error * kP;
        double d = 0;

        if (deltaTime > 0) {
            d = ((error - lastError) / deltaTime) * kD;
        }

        double power;

        if (Math.abs(error) < angleTol) {
            power = 0;
        } else {
            power = Range.clip(p + d, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }
}