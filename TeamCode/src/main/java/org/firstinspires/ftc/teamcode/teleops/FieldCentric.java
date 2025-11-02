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
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainNoPedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

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
    private ElapsedTime runtime = new ElapsedTime();
    public FlywheelSubsystem flywheel;
    public IntakeSubsystem intake;
    //boolean isFlywheelRunning;
    boolean isFlywheelOpen;
    double flywheelVelocity = 0;

    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Configurable
    public static class configurables {
        public static double targetVelocity = 1200;
    }

    @Override
    public void init() {
        if (startingPose == null) {
            startingPose = new Pose(0,0,0);
        }
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        flywheel = new FlywheelSubsystem(hardwareMap);
        flywheel.init();
        intake = new IntakeSubsystem(hardwareMap);
        intake.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
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
        //control for flywheel state
        //flywheelVelocity = configurables.targetVelocity;
        if (gamepad2.y) {
            flywheelVelocity = 1200;
        } else if (gamepad2.a) {
            flywheelVelocity = 0;
        } else if (gamepad2.x) {
            flywheelVelocity = 2150
            ;
        }

        //speed for flywheel state
            flywheel.flywheelMotor.setPower(flywheel.flywheelPIDF.calculate(flywheel.flywheelMotor.getVelocity(), flywheelVelocity));

        //control for servo state
        if (gamepad2.dpad_up) {
            isFlywheelOpen = true;
        } else if (gamepad2.dpad_down) {
            isFlywheelOpen = false;
        }

        //run the intake
        if (gamepad2.right_trigger > 0) {
            intake.setIntakePower(0.35);
        } else if (gamepad2.left_trigger > 0) {
            intake.setIntakePower(-0.35);
        } else {
            intake.setIntakePower(0);
        }

        //change state of servo
        flywheel.manageScoring(isFlywheelOpen);

        /*if (isFlywheelOpen && isFlywheelRunning) {
            flywheel.changePID(2);
        } else {
            flywheel.changePID(1);
        }*/

        //run drivetrain
        pedroDrive();

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //send telemetry
        //telemetry.addData("Flywheel Running: ", isFlywheelRunning);
        telemetry.addData("Flywheel Velocity", flywheel.flywheelMotor.getVelocity());
        telemetry.addData("Servo Open: ", isFlywheelOpen);
        telemetry.addData("Intake Power: ", intake.intakeMotor.getPower());
    }

    public void pedroDrive() {
        follower.update();
        telemetryM.update();

        if(!automatedDrive) {
            if(!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
                    );
            } else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                false
        );

        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
