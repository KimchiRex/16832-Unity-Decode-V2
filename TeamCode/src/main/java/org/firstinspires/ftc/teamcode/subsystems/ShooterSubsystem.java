package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.util.InterpLUT;
//import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ShooterSubsystem implements UnitySubsystem{

    public final HardwareMap hardwareMap;

    public FlywheelSubsystem flywheel;
    public TurretSubsystem turret;
    public InterpLUT veloRegression;
    public InterpLUT angleRegression;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        /* distance notes:

            close closest: 25
            close default:63 (gate cycles)
            close furthest:114

            far closest: 118
            far default: 141 (<- bot touching middle tape basically) 144 (<- bot just a little bit in zone, natural teleop position)
            far default: (their human player teleop)
            far default: (auto)
            far farthest:155

         */
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.add(10,0);
        veloRegression.createLUT();

        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.add(10, 0);
        angleRegression.createLUT();
    }

    @Override
    public void teleOpManual(Gamepad gamepad) {

    }

    public void init() {
        flywheel = new FlywheelSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
    }

    public Vector getShotVector(Pose botPose, Vector targetPose) {
        return new Vector(0,0);
    }

    public void shootStationary(Pose botPose, Vector targetPose) {
        //setup
        Vector shot = getShotVector(botPose, targetPose);

        //regression
        double distanceToTarget = shot.getMagnitude();
        double flywheelTargetVelocity = veloRegression.get(distanceToTarget);
        double hoodTargetAngle = angleRegression.get(distanceToTarget);

        //execution
        turret.setAngle(shot.getTheta() - botPose.getHeading());
        flywheel.runPIDF(flywheel.inPerSecToRPM(flywheelTargetVelocity));
        flywheel.setHoodAngle(hoodTargetAngle);
    }

    public void shootWhileMoving(Pose botPose, Vector targetPose, Vector botVelocity) {
        //setup
        Vector shot = getShotVector(botPose, targetPose).plus(botVelocity);

        //regression ig
        double distanceToTarget = shot.getMagnitude();
        double flywheelTargetVelocity = veloRegression.get(distanceToTarget);
        double hoodTargetAngle = angleRegression.get(distanceToTarget);

        //execution
        turret.setAngle(shot.getTheta() - botPose.getHeading());
        flywheel.runPIDF(flywheel.inPerSecToRPM(0));
        flywheel.setHoodAngle(0);
    }

    public void nullBehavior() {
        flywheel.runMotorsTogether(0);
    }

}
