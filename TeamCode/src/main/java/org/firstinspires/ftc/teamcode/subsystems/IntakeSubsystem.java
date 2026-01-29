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
    public DcMotorEx intakeMotor1;
    public DcMotorEx intakeMotor2;

    public HardwareMap hardwareMap;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void teleOpManual(Gamepad gamepad) {};
    public void init() {
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intakeMotor1");
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intakeMotor2");
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setIntake1(double velocity) {
        intakeMotor1.setVelocity(velocity);
    }
    public void setIntake2(double velocity) {
        intakeMotor2.setVelocity(velocity);
    }

    public void setIntake1off() {
        intakeMotor1.setPower(0);
    }

    public void setIntake2off() {
        intakeMotor2.setPower(0);
    }
}
