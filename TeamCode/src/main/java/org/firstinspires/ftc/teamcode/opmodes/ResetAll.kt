package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem

@ArmSubsystem.Attach
@LiftSubsystem.Attach
@Autonomous(name = "N/A | Auto - N/A | Reset Encoders", group = "2024-25 OpModes")
class ResetAll : OpMode() {
    override fun init() {}

    override fun start() {
        ArmSubsystem.reset()
        LiftSubsystem.reset()
    }

    override fun loop() {}
}