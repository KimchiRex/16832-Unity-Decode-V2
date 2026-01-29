package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class TurretSubsystem implements UnitySubsystem {

    public Servo turretServo1, turretServo2, turretServo3;
    public Encoder absEncoder;
    public HardwareMap hardwareMap;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");
        absEncoder = hardwareMap.get(Encoder.class, "absoluteEncoder");
    }

    public void teleOpManual(Gamepad gamepad) {

    }

    public double absEncoderTicks = absEncoder.getDeltaPosition();
    public double turretHeadingChange = ((absEncoderTicks / 4000) * 360) % 360;

    public void setAngle(double targetAngle) {
        double servoPosition = targetAngle / 360;
        //does some
        turretServo1.setPosition(servoPosition);
        turretServo2.setPosition(servoPosition);
        turretServo2.setPosition(servoPosition);
    }

    public double getAngle(){
        return 0;
    }

}
