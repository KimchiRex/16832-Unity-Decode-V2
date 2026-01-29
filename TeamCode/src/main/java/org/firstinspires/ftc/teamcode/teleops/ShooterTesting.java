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

import static org.firstinspires.ftc.teamcode.teleops.ShooterTesting.FlywheelPIDFS.kS;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.teleops.ShooterTesting.telemetryM;

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
    public FlywheelSubsystem flywheel;

    public IntakeSubsystem intake;
    public double flywheelVelocity;
    public double motorPower;
    public double robotVoltage = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Configurable
    public static class FlywheelPIDFS {
        public static double kS = 0.03, kV, kA;

        public static double speed = 1200;
    }

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheel = new FlywheelSubsystem(hardwareMap);
        flywheel.init();
        intake = new IntakeSubsystem(hardwareMap);
        intake.init();
        // Tell the driver that initialization is complete.
        //telemetryM.debug("idk text here");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        robotVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetryM.debug("flywheel velocity", + flywheelVelocity);
        telemetryM.debug("flywheel power" + motorPower);
        telemetryM.debug("robot voltage:" + robotVoltage);
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
        intake.setIntakePower(.35);
        robotVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();        // Setup a variable for each drive wheel to save power level for telemetry
        //flywheel.flywheelController.calculate(1200);
        flywheelVelocity = flywheel.flywheelMotor1.getVelocity();
        motorPower = runtime.seconds() / 50;
        flywheel.runPIDF();
        telemetryM.debug("flywheel velocity:" + flywheelVelocity);
        telemetryM.debug("flywheel power:" + motorPower);
        telemetryM.debug("robot voltage:" + robotVoltage);
        telemetryM.update(telemetry);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
