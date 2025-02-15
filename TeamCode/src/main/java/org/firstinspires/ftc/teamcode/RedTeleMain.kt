package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.structures.PIDFAdjuster
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FieldCentricDrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftPIDFSubsystem
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem


/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */

@Mercurial.Attach
//@DrivetrainSubsystem.Attach
@FieldCentricDrivetrainSubsystem.Attach
@VisionSubsystem.Attach
@ClawSubsystem.Attach
@LiftPIDFSubsystem.Attach
// @LinearSlidePIDFSubsystem.Attach
@TeleOp(name = "Red | Tele - N/A | Main", group = "2024-25 OpCodes")
class RedTeleMain : OpMode() {
    private lateinit var pidfAdjuster: PIDFAdjuster
    override fun init() {
        telemetry.addData("Status", "Initialized")

        Mercurial.gamepad1.a.onTrue(ClawSubsystem.updateClawStateL(ClawSubsystem.ClawState.OPEN))
        Mercurial.gamepad1.b.onTrue(ClawSubsystem.updateClawStateL(ClawSubsystem.ClawState.CLOSED))

        pidfAdjuster = PIDFAdjuster.createAndAttach(telemetry, Mercurial.gamepad2)
        telemetry.update()
    }

    override fun loop() {
        pidfAdjuster.updateTelemetry()
    }
}