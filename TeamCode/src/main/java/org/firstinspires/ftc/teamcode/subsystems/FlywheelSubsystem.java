package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import java.math.*;

public class FlywheelSubsystem implements UnitySubsystem{

    public static boolean isIntaking;
    public final HardwareMap hardwareMap;
    public DcMotorEx flywheelMotor1, flywheelMotor2;
    public Servo blocker;

    public Servo hood;
    double openPosition = .9;
    double closedPosition = 0.48;

    double currentTargetVelocity;
    double currentVelocity;
    public PIDFController flywheelPIDF = new PIDFController(FlywheelPIDFConstants.kP, FlywheelPIDFConstants.kI, FlywheelPIDFConstants.kD, FlywheelPIDFConstants.kF);





    @Configurable
    public static class FlywheelPIDFConstants {
        public static double kP = 0.005, kI = 0, kD, kF = 0.00042;

        public static double speed = 1150;
    }



    public FlywheelSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void teleOpManual(Gamepad gamepad) {

    }

    public void init() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setDirection(Servo.Direction.FORWARD);
        flywheelMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void runMotorsTogether(double power) {
        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power);
    }

    public void runPIDF() {
        runMotorsTogether(flywheelPIDF.calculate(flywheelMotor1.getVelocity(), FlywheelPIDFConstants.speed));
    }

    public void runPIDF(double targetVelocity) {
        runMotorsTogether(flywheelPIDF.calculate(flywheelMotor1.getVelocity(), targetVelocity));
    }

    public void runLocalizedPIDF(double distanceToTarget) {
        double targetVelocity = distanceToTarget; //make actual formula for conversion
    }
    public void manageScoring(boolean open) {
        if (open) {
            blocker.setPosition(openPosition);
        } else {
            blocker.setPosition(closedPosition);;
        }
    }

    public void runBangBang(double targetVelocity) {
        if (flywheelMotor1.getVelocity() < targetVelocity) {
            runMotorsTogether(1);
        } else {
            runMotorsTogether(0);
        }
    }

    public void setHoodAngle(double hoodAngle) {

    }


    public double inPerSecToRPM(double inPerSec) {
        return 0;
    }

}
