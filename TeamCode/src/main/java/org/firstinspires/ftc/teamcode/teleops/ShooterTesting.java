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

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import static org.firstinspires.ftc.teamcode.teleops.ShooterTesting.telemetryM;

import java.util.List;

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
@TeleOp(name="Shooter Testing", group="Iterative OpMode")
public class ShooterTesting extends OpMode
{
    // Declare OpMode members.
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private ElapsedTime runtime = new ElapsedTime();
    private List<LynxModule> allHubs;
    public FlywheelSubsystem flywheel;

    public IntakeSubsystem intake;
    public TurretSubsystem turret;
    public ShooterSubsystem shooter;
    public DriveSubsystem drivetrain;
    public Vector targetPose;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Configurable
    public static class TuningValues {
        public static double kP, kI, kD, kF;

        public static double flywheelVelocity = 1200;
        public static double shooterAngle = 0;
        public static double initalIntakePower = 1;
        public static double transferPower = 1;

        public static double targetPoseX = 12;
        public static double targetPoseY = 134;
    }

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheel = new FlywheelSubsystem(hardwareMap);
        flywheel.init();
        intake = new IntakeSubsystem(hardwareMap);
        intake.init();
        turret = new TurretSubsystem(hardwareMap);
        turret.init();
        drivetrain = new DriveSubsystem(hardwareMap);
        drivetrain.init();
        drivetrain.setPose(new Pose(70.75, 70.75, Math.toRadians(90)));
        shooter = new ShooterSubsystem(hardwareMap);

        targetPose = new Vector(TuningValues.targetPoseX, TuningValues.targetPoseY);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetryM.update(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();  // Once per loop
        }

        drivetrain.updateOdometry();

        turret.updatePosition(drivetrain.getPose(), new Pose());

        flywheel.runPIDF(TuningValues.flywheelVelocity);

        if (gamepad1.right_trigger > 0.05) {
            intake.setPowerInitialIntake(TuningValues.initalIntakePower);
            intake.setPowerTransfer(TuningValues.transferPower);
        } else {
            intake.turnOffFloat();
        }

        telemetryM.debug("flywheel velocity:" + flywheel.flywheelMotor1.getVelocity());
        telemetryM.debug("flywheel power:" + flywheel.flywheelMotor1.getPower());
        telemetryM.debug("robot x: ", drivetrain.getPose().getX());
        telemetryM.debug("robot y: ", drivetrain.getPose().getY());
        telemetryM.debug("robot heading: ", drivetrain.getPose().getHeading());
        telemetryM.debug("distance to target: ", shooter.getShotVector(drivetrain.getPose(), targetPose));

        telemetryM.update(telemetry);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
