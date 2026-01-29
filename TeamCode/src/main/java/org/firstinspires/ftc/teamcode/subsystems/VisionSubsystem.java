package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionSubsystem{

    public Limelight3A limelight;
    public HardwareMap hardwareMap;
    public Pose3D LLEstimateMT2;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void runLimelight(double robotYaw) {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            LLEstimateMT2 = result.getBotpose_MT2();
            positionEstimate = new Vector(LLEstimateMT2.getPosition().x, LLEstimateMT2.getPosition().y);
            canSee = true;
        } else {
            canSee = false;
        }
    }

    public boolean canSee = false;
    public Vector positionEstimate;

    public double getDistanceFromTarget() {
        return 0;
    }

    public double getAngleToTarget() {
        return 0;
    }
}
