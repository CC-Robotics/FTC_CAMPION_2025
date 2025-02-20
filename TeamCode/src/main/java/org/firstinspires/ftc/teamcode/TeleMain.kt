package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.structures.PIDFAdjuster
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlideSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlideSubsystem.SlidePosition
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem
import vision.PolishedSampleDetection


/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */

@Mercurial.Attach
@DrivetrainSubsystem.Attach
//@FieldCentricDrivetrainSubsystem.Attach
@VisionSubsystem.Attach
@ClawSubsystem.Attach
@LiftSubsystem.Attach
@LinearSlideSubsystem.Attach
open class TeleMain : OpMode() {
    private lateinit var pidfAdjuster: PIDFAdjuster
    override fun init() {
        telemetry.addData("Status", "Initialized")

        Mercurial.gamepad1.a.onTrue(ClawSubsystem.updateClawStateL(ClawSubsystem.ClawState.OPEN))
        Mercurial.gamepad1.b.onTrue(ClawSubsystem.updateClawStateL(ClawSubsystem.ClawState.CLOSED))

        pidfAdjuster = PIDFAdjuster(telemetry, Mercurial.gamepad2)
        pidfAdjuster.attach()
        telemetry.update()
    }

    override fun loop() {
        when (Config.behavior) {
            Config.Behavior.MANUAL -> {
                LiftSubsystem.update(Mercurial.gamepad2.leftStickY.state)
                LinearSlideSubsystem.update(Mercurial.gamepad2.rightStickY.state)
            }

            Config.Behavior.COLLECTING -> {
                Mercurial.gamepad2.a.onTrue(Lambda("Set to manual").addExecute {
                    Config.behavior = Config.Behavior.MANUAL
                })
                val sample = VisionSubsystem.getBestContour() ?: return
            }

            Config.Behavior.RUN_TO_VISION_POSITION -> {
                LinearSlideSubsystem.setPosition(SlidePosition.VISION_POSITION)
                if (LinearSlideSubsystem.isBasicallyAt(SlidePosition.VISION_POSITION)) {
                    Config.behavior = Config.Behavior.COLLECTING
                }
            }
        }
        pidfAdjuster.updateTelemetry()
    }
}

@TeleOp(name = "Red | Tele - N/A | Main", group = "2024-25 OpCodes")
class RedTeleMain : TeleMain() {
    override fun init() {
        super.init()
        Config.allianceColour = Config.SampleColor.RED
    }
}

@TeleOp(name = "Blue | Tele - N/A | Main", group = "2024-25 OpCodes")
class BlueTeleMain : TeleMain() {
    override fun init() {
        super.init()
        Config.allianceColour = Config.SampleColor.BLUE
    }
}