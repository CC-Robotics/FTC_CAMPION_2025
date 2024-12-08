package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftPIDFSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem

/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */

@Mercurial.Attach
@DrivetrainSubsystem.Attach
@ClawSubsystem.Attach
@LiftPIDFSubsystem.Attach
@TeleOp(name = "Campion TeleOp 2024-25 (2)", group = "Linear OpMode")
class TeleOpMode : OpMode() {
    override fun init() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        Mercurial.gamepad1.a.onTrue(ClawSubsystem.open(telemetry))
        Mercurial.gamepad1.b.onTrue(ClawSubsystem.close(telemetry))

        Mercurial.gamepad1.dpadUp.onTrue(LiftSubsystem.goUp(telemetry))
        Mercurial.gamepad1.dpadDown.onTrue(LiftSubsystem.goDown(telemetry))
    }

    override fun loop() {}
}