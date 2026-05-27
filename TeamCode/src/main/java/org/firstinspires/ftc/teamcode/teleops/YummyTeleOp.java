package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="TeleOp", group="Iterative OpMode")
public class YummyTeleOp extends OpMode
{
    // Declare OpMode members.

    //dt object, contains odo & dt code

    //intake object

    //turret & shooter


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");


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
        runtime.reset();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
    }

}
