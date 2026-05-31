package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveSubsystem implements UnitySubsystem {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private HardwareMap hardwareMap;
    private MecanumDrive drive;
    private PIDFController turnPID = new PIDFController(0,0,0,0);

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        //init this shit
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backLeft = new Motor(hardwareMap, "backLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backRight = new Motor(hardwareMap, "backRight");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        pinpoint.setOffsets(-84,-136, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void teleOpManual(Gamepad gamepad) {

    }

    public Pose getPose() {
        return new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );
    }

    public void setPose(Pose pose) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()));
    }

    public void updateOdometry() {
        pinpoint.update();
    }

    public void runDrivetrainRobotCentric(Gamepad gamepad) {
        drive.driveRobotCentric(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x);
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
