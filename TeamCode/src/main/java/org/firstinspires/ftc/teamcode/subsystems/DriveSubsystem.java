package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveSubsystem implements UnitySubsystem {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private HardwareMap hardwareMap;
    private MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    private PIDFController turnPID = new PIDFController(0,0,0,0);

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        //init this shit
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        frontLeft = hardwareMap.get(Motor.class, "frontLeft");
        frontRight = hardwareMap.get(Motor.class, "frontRight");
        backLeft = hardwareMap.get(Motor.class, "backLeft");
        backRight = hardwareMap.get(Motor.class, "backRight");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0,0, DistanceUnit.MM);
    }

    public void teleOpManual(Gamepad gamepad) {

    }

    public Pose getPose() {
        return new Pose();
    }

    public void setPose(Pose pose) {}

    public void updateOdometry() {

    }

    public void runDrivetrainRobotCentric(Gamepad gamepad) {
        drive.driveRobotCentric(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void runDrivetrainRobotCentric(double strafe, double forward, double turn) {
        drive.driveRobotCentric(strafe, forward, turn);
    }

    public double getTurnPID(double robotHeading, double targetHeading) {
        return turnPID.calculate(targetHeading - robotHeading);
    }
    public double getAngle(){
        return 0;
    }

}
