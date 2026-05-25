package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LLVisionSubsystem;

public class ShooterSubsystem implements UnitySubsystem{

    public final HardwareMap hardwareMap;

    public FlywheelSubsystem flywheel;
    public TurretSubsystem turret;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void teleOpManual(Gamepad gamepad) {

    }

    public void init() {
        flywheel = new FlywheelSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
    }

    public Vector shotVector(Pose botPose, Vector targetPose) {
        return new Vector(0,0);
    }

    public void shootStationary(Pose botPose, Vector targetPose) {
        //setup
        Vector shot = shotVector(botPose, targetPose);
        //math


        //execution
        turret.setAngle(shot.getTheta() - botPose.getHeading());
        flywheel.runPIDF(flywheel.inPerSecToRPM(0));
        flywheel.setHoodAngle(0);
    }

    public void shootWhileMoving(Pose botPose, Vector targetPose, Vector botVelocity) {
        //setup
        Vector shot = shotVector(botPose, targetPose).plus(botVelocity);
        //math


        //execution
        turret.setAngle(shot.getTheta() - botPose.getHeading());
        flywheel.runPIDF(flywheel.inPerSecToRPM(0));
        flywheel.setHoodAngle(0);
    }

    public void nullBehavior() {
        flywheel.runMotorsTogether(0);
    }

}
