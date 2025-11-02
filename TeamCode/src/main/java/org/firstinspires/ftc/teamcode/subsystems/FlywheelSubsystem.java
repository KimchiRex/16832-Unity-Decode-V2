package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class FlywheelSubsystem implements UnitySubsystem{

    public static boolean isIntaking;
    public final HardwareMap hardwareMap;
    public DcMotorEx flywheelMotor;
    public Servo intakeStop;
    double openPosition = .8;
    double closedPosition = 0.48;
    public PIDFController flywheelPIDF = new PIDFController(FlywheelPIDFConstants.kP, FlywheelPIDFConstants.kI, FlywheelPIDFConstants.kD, FlywheelPIDFConstants.kF);

    public void runPIDF() {
        flywheelMotor.setPower(flywheelPIDF.calculate(flywheelMotor.getVelocity(), 1200));
    }

    @Configurable
    public static class FlywheelFeedforwardConstants {
        public static double kS= 0.025, kV = 0.00004, kA;
    }

    @Configurable
    public static class FlywheelPIDFConstants {
        public static double kP = 0.005, kI = 0, kD, kF = 0.00042;
    }



    public FlywheelSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

    }

    @Override
    public void teleOpManual(Gamepad gamepad) {

    }

    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeStop = hardwareMap.get(Servo.class, "intakeStop");
        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public double currentSpeed() {
        return flywheelMotor.getVelocity();
    }
    public void setSpeed(double speed) {
        flywheelMotor.setVelocity(speed);
    }

    public void manageScoring(boolean open) {
        if (open) {
            intakeStop.setPosition(openPosition);
        } else {
            intakeStop.setPosition(closedPosition);
        }
    }

    public void changePID(double P) {
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(P,0,0,0));
    }

    public boolean inRange(double velocity, double acceleration, double desiredVelocity) {
        double timeToRelease = .3;
        return (velocity + acceleration * timeToRelease) > desiredVelocity - 10 && (velocity + acceleration * timeToRelease) > desiredVelocity + 10;
    }





}
