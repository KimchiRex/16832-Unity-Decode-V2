package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class TurretSubsystem implements UnitySubsystem {

    public Servo turretServo1, turretServo2;
    public HardwareMap hardwareMap;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
    }

    public void teleOpManual(Gamepad gamepad) {

    }

    public void setAngle(double targetAngle) {
        double servoPosition = targetAngle / 360;
        //does some
        turretServo1.setPosition(servoPosition);
        turretServo2.setPosition(servoPosition);
    }

    public void updatePosition(Pose robotPose, Pose targetPose) {
        //do something ig
    }

    public double getAngle(){
        return 0;
    }

}
