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
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.List;

//testing our teleop localization, likely using pose values for auto pathing
@Configurable
@TeleOp(name="Shooter Testing", group="Iterative OpMode")
public class NoPedroLocalizationTesting extends OpMode
{
    // Declare OpMode members.
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private ElapsedTime runtime = new ElapsedTime();
    private List<LynxModule> allHubs;
    public DriveSubsystem drivetrain;

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
        drivetrain.init();
        drivetrain.setPose(new Pose(70.75, 70.75, Math.toRadians(90)));
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
        drivetrain.runDrivetrainRobotCentric(gamepad1);

        if (gamepad1.dpadDownWasPressed()) {
            drivetrain.setPose(new Pose(70.75, 70.75, Math.toRadians(90)));
        }

        telemetryM.debug("robot x: ", drivetrain.getPose().getX());
        telemetryM.debug("robot y: ", drivetrain.getPose().getY());
        telemetryM.debug("robot heading: ", drivetrain.getPose().getHeading());

        telemetryM.update(telemetry);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
