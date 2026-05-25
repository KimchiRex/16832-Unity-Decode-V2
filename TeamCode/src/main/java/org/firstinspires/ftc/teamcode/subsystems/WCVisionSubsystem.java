package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class WCVisionSubsystem {

    public Limelight3A limelight;
    public HardwareMap hardwareMap;
    public Pose3D LLEstimateMT2;

    public WCVisionSubsystem(HardwareMap hardwareMap) {
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
