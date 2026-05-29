package org.firstinspires.ftc.teamcode.autos; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "Blue Close Solo", group = "Autos")
public class BlueSolo21 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public FlywheelSubsystem flywheel;
    public IntakeSubsystem intake;

    private final Pose startPose = new Pose(48, 134, Math.toRadians(-90));
    private final Pose scorePreloadPose = new Pose(60, 85, Math.toRadians(136.5));
    private final Pose scorePose= new Pose();
    private final Pose pickup1Pose = new Pose(20, 82, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(16, 56.5, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(18.5, 68, Math.toRadians(180));
    private final Pose intakeGate = new Pose(13, 56, Math.toRadians(155));
    private final Pose leavePointPrepareTeleOp = new Pose(50, 65, Math.toRadians(180));
    private Path scorePreload;
    private PathChain grabSecondRow, scoreSecondRow, grabRamp, scoreRamp, grabFirstRow, scoreFirstRow, extraHit;
    private double intaking = 0;
    private boolean oncePer = true;
    private boolean flywheelRunning = true;
    private double shootingVelocity;

    public void buildPaths() {

    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        //movingOn = false;
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("actionTimer", actionTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        flywheel = new FlywheelSubsystem(hardwareMap);
        flywheel.init();
        intake = new IntakeSubsystem(hardwareMap);
        intake.init();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        flywheel.manageScoring(false);
        shootingVelocity = 1100;
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        flywheel.manageScoring(false);
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Constants.endAutoPose = follower.getPose();
    }
    public void scoreArtifactsClose(PathChain nextPath, int numNextPath) {
        if (follower.getDistanceTraveledOnPath() > 15 && pathTimer.getElapsedTimeSeconds() > 0.3) {
            intaking = 0;
        }

        if (follower.getDistanceRemaining() < 5 && pathTimer.getElapsedTimeSeconds() > 0.3) {
            flywheel.manageScoring(true);
        }
        if(!follower.isBusy()) {
            if (oncePer) {
                intaking = 1;
                actionTimer.resetTimer();
                oncePer = false;
            }

            if(actionTimer.getElapsedTimeSeconds() > .7) {
                flywheel.manageScoring(false);
                follower.followPath(nextPath);
                setPathState(numNextPath);
                oncePer = true;
            }
        }
    }

    public void intakeArtifacts(PathChain nextPath, int numNextPath) {
        intaking = 1;
        if (follower.getDistanceTraveledOnPath() > 5) {
            flywheel.manageScoring(false);
        }
        if(!follower.isBusy()) {
            follower.followPath(nextPath);
            intaking= 0.8;
            setPathState(numNextPath);
            oncePer = true;
        }
    }
}