package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;

@TeleOp(name="TeleOp", group="Iterative OpMode")
public class YummyTeleOp extends OpMode
{
    //subsystems and objects ig
    public DriveSubsystem drivetrain;
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;
    private List<LynxModule> allHubs;
    private ElapsedTime runtime = new ElapsedTime();

    //variables and constants
    public Vector targetPose = new Vector();
    public Pose closeRelocalizationPose;
    public Pose farRelocalizationPose;

    public double gateIntakeAngle;

    //enum bc y not ig
    public enum RobotState {
        SEARCHING,
        //GATE_SEARCHING,
        INTAKING,
        //GATE_INTAKING,
        TRANSPORTING,
        //GATE_TRANSPORTING,
        SHOOTING
    }

    public enum ScoringPosition {
        CLOSE,
        FAR,
    }

    public RobotState currentRobotState;
    public ScoringPosition currentScoringPosition;


    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DriveSubsystem(hardwareMap);
        drivetrain.init();
        if (Constants.endAutoPose != null) {
            drivetrain.setPose(Constants.endAutoPose);
        } else {
            drivetrain.setPose(new Pose(70.75, 70.75, Math.toRadians(90)));
        }

        shooter = new ShooterSubsystem(hardwareMap);
        shooter.init();

        intake = new IntakeSubsystem(hardwareMap);
        intake.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Blue < | > Red\n");
        if (Constants.goalSide == Constants.GoalSide.NOTSET) {
            telemetry.addLine("Target Goal Not Set");
        } else if (Constants.goalSide == Constants.GoalSide.RED) {
            telemetry.addLine("Target Goal: RED");
        } else {
            telemetry.addLine("Target Goal: Blue");
        }
    }

    @Override
    public void start() {
        if (Constants.goalSide == Constants.GoalSide.NOTSET) {
            Constants.goalSide = Constants.GoalSide.BLUE;
        }

        //set reference positions for teleop
        if (Constants.goalSide == Constants.GoalSide.RED) {
            targetPose.setOrthogonalComponents(141.5-12, 134);
            closeRelocalizationPose = new Pose(141.5 - 15,82.5,0);
            farRelocalizationPose = new Pose(141.5 - 9,8,180);
            gateIntakeAngle = Math.toRadians(150);
        } else {
            targetPose.setOrthogonalComponents(12, 134);
            closeRelocalizationPose = new Pose(15,82.5,180);
            farRelocalizationPose = new Pose(9,8,0);
            gateIntakeAngle = Math.toRadians(30);
        }

        runtime.reset();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();  // Once per loop
        }

        drivetrain.updateOdometry();
        drivetrain.runDrivetrainRobotCentric(gamepad1);

        //determine what state the robot is in based on driver control

        //execute based on robot state
        switch (currentRobotState) {
            case SEARCHING:

                shooter.flywheel.manageScoring(false);
                intake.turnOffFloat();
                shooter.shootStationary(drivetrain.getPose(), targetPose);
                break;

            case TRANSPORTING:

                shooter.flywheel.manageScoring(true);
                intake.turnOffBrake();
                shooter.shootStationary(drivetrain.getPose(), targetPose);
                break;

            case INTAKING:

                shooter.flywheel.manageScoring(false);
                intake.setPowerInitialIntake(1);
                intake.setPowerTransfer(intake.getBallStoredInTransfer() ? 0 : 1);
                shooter.shootStationary(drivetrain.getPose(), targetPose);
                break;

            case SHOOTING:
                shooter.flywheel.manageScoring(true);
                intake.setPowerInitialIntake(1);
                intake.setPowerTransfer(1);
                shooter.shootStationary(drivetrain.getPose(), targetPose);
                break;
        }


    }

    @Override
    public void stop() {
    }

}
