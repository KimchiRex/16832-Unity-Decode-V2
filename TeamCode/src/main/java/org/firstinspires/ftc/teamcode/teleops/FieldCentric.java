/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LLVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WCVisionSubsystem;

import java.util.function.Supplier;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Configurable
@TeleOp(name="Field-Centric TeleOp", group="Iterative OpMode")
public class FieldCentric extends OpMode
{
    // Declare OpMode members.
    private Follower follower;
    public ShooterSubsystem shooter;
    public LLVisionSubsystem vision;
    public IntakeSubsystem intake;
    public static Pose startingPose;
    public static Vector targetVector;
    private boolean blockerOpen = false;
    private boolean shooterRunning = false;

    public enum SideColor {
        RED, BLUE
    }

    SideColor sideColor = SideColor.BLUE;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.init();
        vision = new LLVisionSubsystem(hardwareMap);
        vision.init();
        intake = new IntakeSubsystem(hardwareMap);
        intake.init();
        startingPose = Constants.startingPose;
        if (startingPose == null) {
            startingPose = new Pose(0,0,0);
        }
        follower.setStartingPose(startingPose);
        follower.update();
        Constants.driveConstants.setUseBrakeModeInTeleOp(true);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.leftBumperWasPressed()) {
            sideColor = SideColor.BLUE;
        } else if (gamepad1.rightBumperWasPressed()) {
            sideColor = SideColor.RED;
        }

        if(sideColor == SideColor.BLUE) {
            targetVector = new Vector(0,0);
            telemetry.addLine("Side Color: BLUE");
        } else {
            targetVector = new Vector(0,0);
            telemetry.addLine("Side Color: RED");
        }

        telemetry.addLine("\n=== LB: Blue, RB: Red ===");
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //run dt
        pedroDrive();

        //vision and pose logic
        vision.runLimelight(follower.getHeading() + shooter.turret.getAngle());
        if (vision.canSee) {
            //updates pose
            if (gamepad1.rightBumperWasPressed()) follower.setPose(new Pose(
                    vision.positionEstimate.getXComponent(), vision.positionEstimate.getYComponent(), follower.getHeading()));
        }
        //in case something fucks up
        if (gamepad1.a && gamepad1.dpad_down) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(90)));
        }

        //intaking logic
        if (gamepad1.right_trigger > 0.5) {
            if (blockerOpen) {
                intake.setIntake1(400);
                intake.setIntake2(400);
            } else {
                intake.setIntake1(400);
                intake.setIntake2off();
            }
        } else {
            intake.setIntake1off();
            intake.setIntake2off();
        }

        //shooting logic
        if (gamepad1.right_bumper) {
            blockerOpen = true;
        } else {
            blockerOpen = false;
        }

        if (gamepad1.left_bumper) {
            shooterRunning = !shooterRunning;
        }

        if (shooterRunning) {
            shooter.shootWhileMoving(follower.getPose(), targetVector, follower.getVelocity());
        } else {
            shooter.nullBehavior();
        }
    }

    public void pedroDrive() {
        follower.update();

        if (sideColor == SideColor.BLUE) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        } else {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }
        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
