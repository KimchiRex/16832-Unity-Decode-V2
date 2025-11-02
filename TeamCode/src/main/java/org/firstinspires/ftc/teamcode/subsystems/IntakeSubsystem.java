package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class IntakeSubsystem implements UnitySubsystem {

    //@Override
    public DcMotorEx intakeMotor;

    public HardwareMap hardwareMap;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void teleOpManual(Gamepad gamepad) {};
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSpeed(double speed) {
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setVelocity(speed);
    }

    public void intake(String intaking) {
        if (intaking == "in") {
            setSpeed(10);
        } else if (intaking == "not"){
            setSpeed(0);
        } else if (intaking == "out") {
            setSpeed(-10);
        }
    }
    public void passOne() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition() + 500);
        intakeMotor.setPower(.3);
    }

    public void setIntakePower(double power) {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setPower(power);
    }

    public void intakeAuto() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setPower(0.5);
    }

    public void intakeOffAuto() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setPower(0);
    }
}
