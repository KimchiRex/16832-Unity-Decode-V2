package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

interface UnitySubsystem {
    //public abstract void teleOpSample(Gamepad gamepad);
    //public abstract void teleOpSpecimen(Gamepad gamepad);
    public abstract void teleOpManual(Gamepad gamepad);
    public abstract void init();
}
