package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.structures.PIDFAdjuster
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftPIDFSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlidePIDFSubsystem

/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */

@Mercurial.Attach
//@DrivetrainSubsystem.Attach
@ClawSubsystem.Attach
@LiftPIDFSubsystem.Attach
@LinearSlidePIDFSubsystem.Attach
@TeleOp(name = "Red | Tele - N/A | Main", group = "2024-25 OpCodes")
class RedTeleMain : OpMode() {
    override fun init() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        Mercurial.gamepad1.a.onTrue(ClawSubsystem.open(telemetry))
        Mercurial.gamepad1.b.onTrue(ClawSubsystem.close(telemetry))

        Mercurial.gamepad1.dpadUp.onTrue(LiftPIDFSubsystem.changePosition(telemetry, 1))
        Mercurial.gamepad1.dpadDown.onTrue(LiftPIDFSubsystem.changePosition(telemetry, -1))

        Mercurial.gamepad1.dpadLeft.onTrue(LinearSlidePIDFSubsystem.changePosition(telemetry))
        Mercurial.gamepad1.dpadRight.onTrue(LinearSlidePIDFSubsystem.changePosition(telemetry, -1))

        //Mercurial.gamepad1.leftBumper.onTrue(LiftPIDFSubsystem.changeDerivative(telemetry, -1))
        //Mercurial.gamepad1.rightBumper.onTrue(LiftPIDFSubsystem.changeDerivative(telemetry, 1))

        PIDFAdjuster.attach(telemetry, Mercurial.gamepad2)
    }

    override fun loop() {}
}