package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretMechanism {
    private DcMotorEx turret;

    private double kP = 0.000001;
    private double kD = 0;

    private double goalX = 0;

    private double lastError = 0;
    private double angleTol = 0.2;
    private final double Max_Power = 0.8;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class,"turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP){
        kP = newKP;
    }
    public double getkP(){
        return kP;
    }
    public void setkD(double newKD){
        kP = newKD;
    }
    public double getkD(){
        return kD;
    }
    public void restartTimer(){
        timer.reset();
    }

    public void update(AprilTagDetection curId){
        double deltaTime = timer.seconds();
        timer.reset();

        if (curId == null){
            turret.setPower(0);
            lastError = 0;
        }

        double error = goalX - curId.ftcPose.bearing;
        double pTerm = error*kP;

        double dTerm = 0;
        if(deltaTime>0){
            dTerm = ((error-lastError) / deltaTime) * kD;
        }
        if(Math.abs(error) < angleTol){
            power = 0;
        }else{
            power = Range.clip(pTerm + dTerm,-Max_Power,Max_Power);
        }

        turret.setPower(power);
        lastError = error;
    }
}


