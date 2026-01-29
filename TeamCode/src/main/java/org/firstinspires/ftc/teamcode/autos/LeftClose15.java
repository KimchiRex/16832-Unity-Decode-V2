package org.firstinspires.ftc.teamcode.autos; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "Left Close 15", group = "Autos")
public class LeftClose15 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public FlywheelSubsystem flywheel;
    public IntakeSubsystem intake;

    private final Pose startPose = new Pose(144-115.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(144-84, 85, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(144-124, 82, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(144-129, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose openGate = new Pose(144-129,75, Math.toRadians(90));
    private final Pose pickup3Pose = new Pose(144-129, 33, Math.toRadians(180));

    private final Pose pickup4plusPose = new Pose(144-129, 20, Math.toRadians(180));
    private final Pose leavePointPrepareTeleOp = new Pose(144-130, 10, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, emptyRamp, scorePickup3, grabHumanPlayer, scoreHumanPlayer, leaveLaunchZone;

    private boolean movingOn = false;
    private double intaking = 0;
    private boolean oncePer = true;
    private boolean flywheelRunning = true;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.25)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(openGate, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(),1,.5)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(63, 57), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading(),0.5)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(144-78,25), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(),.5)
                .build();

        emptyRamp = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(144-100, 85), openGate))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), openGate.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose (144-74,20), pickup4plusPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4plusPose.getHeading(), 0.7)
                .build();

        leaveLaunchZone = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(144-130, 10), leavePointPrepareTeleOp))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePointPrepareTeleOp.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                //go to scoring position while intaking lightly to keep balls in
                intaking = 0.3;
                actionTimer.resetTimer();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                scoreArtifactsClose(grabPickup1, 2);
                break;
            case 2:
                intakeArtifacts(emptyRamp, 6);
                break;
            case 3:
                scoreArtifactsClose(grabPickup2, 4);
                break;
            case 4:
                intakeArtifacts(scorePickup2, 5);
                break;
            case 5:
                scoreArtifactsClose(grabPickup3, 7);
                break;
            case 6:
                intaking = 0.3;
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(3);
                }
                break;
            case 7:
                intakeArtifacts(scorePickup3, 8);
                break;
            case 8:
                scoreArtifactsClose(grabHumanPlayer, 9);
                break;
            case 9:
                intakeArtifacts(scoreHumanPlayer, 10);
                break;
            case 10:
                scoreArtifactsClose(grabHumanPlayer, -1);
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        //actionTimer.resetTimer();
        //movingOn = false;
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        if (flywheelRunning) {
            flywheel.runPIDF();
        } else {
            flywheel.flywheelMotor.setVelocity(0);
        }
        intake.setIntakePower(intaking);
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Constants.startingPose = follower.getPose();
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
    public void stop() {}

    public void scoreArtifactsClose(PathChain nextPath, int numNextPath) {
        if (oncePer) intaking = 0.3;
        if(!follower.isBusy()) {
            if (oncePer) {
                intaking = 0;
                actionTimer.resetTimer();
                oncePer = false;
            }
            if (actionTimer.getElapsedTimeSeconds() > .25 && !movingOn) flywheel.manageScoring(true);

            if (actionTimer.getElapsedTimeSeconds() > .5 && !movingOn) {//1
                intaking = 0.5;
                actionTimer.resetTimer();
                movingOn = true;
            }

            if(movingOn && actionTimer.getElapsedTimeSeconds() > 1.5//2
            ) {
                intaking = 0;
                flywheel.manageScoring(false);
                follower.followPath(nextPath, true);
                setPathState(numNextPath);
                movingOn = false;
                oncePer = true;
            }
        }
    }

    public void intakeArtifacts(PathChain nextPath, int numNextPath) {
        intaking = 0.4;
        if(!follower.isBusy()) {
            follower.followPath(nextPath,true);
            intaking= 0.3;
            setPathState(numNextPath);
        }
    }
}