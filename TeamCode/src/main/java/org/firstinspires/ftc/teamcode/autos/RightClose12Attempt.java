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

@Autonomous(name = "Right Close 12", group = "Examples")
public class RightClose12Attempt extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public FlywheelSubsystem flywheel;
    public IntakeSubsystem intake;

    private final Pose startPose = new Pose(115.5, 128, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84, 85, Math.toRadians(42)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(126, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(129, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose openGate = new Pose(129,70, Math.toRadians(-90));
    private final Pose pickup3Pose = new Pose(129, 36, Math.toRadians(0));
    private final Pose leavePointPrepareTeleOp = new Pose(130, 10, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, emptyRamp, scorePickup3, leaveLaunchZone;

    private boolean movingOn = false;
    private double intaking = 0;
    private boolean oncePer = true;
    private boolean flywheelRunning = true;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                //.addParametricCallback(.3, intake::intakeAuto)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.25)
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(144 - 63, 57), pickup2Pose))
                //.addParametricCallback(.3, intake::intakeAuto)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading(),0.5)
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(81,30), pickup3Pose))
                //.addParametricCallback(.3, intake::intakeAuto)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        emptyRamp = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(100, 73), openGate))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), openGate.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        leaveLaunchZone = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, new Pose(130, 10), leavePointPrepareTeleOp))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePointPrepareTeleOp.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                intaking = 0.3;
                actionTimer.resetTimer();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (oncePer) {
                        intaking = 0;
                        actionTimer.resetTimer();
                        oncePer = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.3 && !movingOn) {
                        flywheel.manageScoring(true);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1 && !movingOn) {
                        intaking = 0.5;
                        actionTimer.resetTimer();
                        movingOn = true;
                    }

                    if(movingOn && actionTimer.getElapsedTimeSeconds() > 3) {
                        intaking = 0;
                        flywheel.manageScoring(false);
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                        movingOn = false;
                        oncePer = true;
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intaking = 0.4;
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    intaking = 0.3;
                    setPathState(3);
                }
                break;
            case 3:
                if (oncePer) {
                    intaking = 0.3;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (oncePer) {
                        intaking = 0;
                        actionTimer.resetTimer();
                        oncePer = false;
                    }
                    flywheel.manageScoring(true);
                    if (actionTimer.getElapsedTimeSeconds() > 1 && !movingOn) {
                        intaking = 0.5;
                        actionTimer.resetTimer();
                        movingOn = true;
                    }

                    if(movingOn && actionTimer.getElapsedTimeSeconds() > 3) {
                        intaking = 0;
                        flywheel.manageScoring(false);
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                        movingOn = false;
                        oncePer = true;
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                intaking = 0.4;
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    intaking= 0.3;
                    setPathState(5);
                }
                break;
            case 5:
                if (oncePer) intaking = 0.3;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (oncePer) {
                        intaking = 0;
                        actionTimer.resetTimer();
                        oncePer = false;
                    }
                    flywheel.manageScoring(true);
                    if (actionTimer.getElapsedTimeSeconds() > 1 && !movingOn) {
                        intaking = 0.5;
                        actionTimer.resetTimer();
                        movingOn = true;
                    }

                    if(movingOn && actionTimer.getElapsedTimeSeconds() > 3) {
                        intaking = 0.4;
                        flywheel.manageScoring(false);
                        follower.followPath(grabPickup3, true);
                        setPathState(7);
                        movingOn = false;
                        oncePer = true;
                    }
                }
                break;
            /*case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position
                intaking = 0;
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;*/
            case 7:
                intaking = 0.4;
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(emptyRamp, true);
                    if (actionTimer.getElapsedTimeSeconds() > 3) setPathState(8);

                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    follower.followPath(scorePickup3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (oncePer) intaking = 0.3;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (oncePer) {
                        intaking = 0;
                        actionTimer.resetTimer();
                        oncePer = false;
                    }
                    flywheel.manageScoring(true);
                    if (actionTimer.getElapsedTimeSeconds() > 1 && !movingOn) {
                        intaking = 0.5;
                        actionTimer.resetTimer();
                        movingOn = true;
                    }

                    if(movingOn && actionTimer.getElapsedTimeSeconds() > 3) {
                        intaking = 0.4;
                        flywheel.manageScoring(false);
                        follower.followPath(leaveLaunchZone, true);
                        setPathState(7);
                        movingOn = false;
                        oncePer = true;
                    }
                }
                break;
            case 10:
                intaking = 0;
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intaking = 0;
                    setPathState(-1);
                }
                break;
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


}