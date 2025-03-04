package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import org.firstinspires.ftc.teamcode.Config.Behavior.*
import org.firstinspires.ftc.teamcode.subsystem.HandSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem


/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */


@Mercurial.Attach
@DrivetrainSubsystem.Attach
@VisionSubsystem.Attach
@HandSubsystem.Attach
@ArmSubsystem.Attach
@LiftSubsystem.Attach
open class TeleMain : OpMode() {
    private lateinit var keybinds: KeybindTemplate

    override fun init() {
        telemetry.addData("Status", "Initialized")

        keybinds = TwoDriverTemplate(Mercurial.gamepad1, Mercurial.gamepad2)

        ArmSubsystem.defaultCommand = ArmSubsystem.update(keybinds)
        LiftSubsystem.defaultCommand = LiftSubsystem.update(keybinds)
        DrivetrainSubsystem.defaultCommand = DrivetrainSubsystem.driveByGamepad(keybinds)

        keybinds.togglePIDF.onTrue(Lambda("Toggle PIDF").addExecute {
            Config.usePIDF = !Config.usePIDF
        })

        keybinds.toggleClaw.onTrue(HandSubsystem.toggleClaw())

        keybinds.toggleCollection.onTrue(Lambda("Toggle collection").addExecute {
            when (Config.behavior) {
                MANUAL -> Config.behavior = COLLECTING
                COLLECTING -> Config.behavior = MANUAL
                RUN_TO_VISION_POSITION -> {}
            }
        })

        keybinds.axleDown.whileTrue(HandSubsystem.incrementPosition(HandSubsystem.ServoType.AXLE, -0.005))
        keybinds.axleUp.whileTrue(HandSubsystem.incrementPosition(HandSubsystem.ServoType.AXLE, 0.005))

        keybinds.wristUp.whileTrue(HandSubsystem.incrementPosition(HandSubsystem.ServoType.WRIST, 0.005))
        keybinds.wristDown.whileTrue(
            HandSubsystem.incrementPosition(
                HandSubsystem.ServoType.WRIST, -0.005
            )
        )

        keybinds.ideallyExtend.onTrue(
            Parallel(
                ArmSubsystem.goTo(190), LiftSubsystem.goTo(3167)
            )
        )

        keybinds.resetEncoder.onTrue(ArmSubsystem.recalibrateEncoders())
    }

    override fun loop() {
        when (Config.behavior) {
            MANUAL -> {}
            COLLECTING -> { Config.behavior = MANUAL}
            RUN_TO_VISION_POSITION -> { Config.behavior = MANUAL }
        }
    }
}

@TeleOp(name = "Red Tele 2", group = "2024-25 OpCodes")
class RedTeleMain : TeleMain() {
    override fun init() {
        super.init()
        Config.allianceColour = Config.SampleColor.RED
    }
}

@TeleOp(name = "Blue Tele 2", group = "2024-25 OpCodes")
class BlueTeleMain : TeleMain() {
    override fun init() {
        super.init()
        Config.allianceColour = Config.SampleColor.BLUE
    }
}

/*
Mercurial.gamepad1.x.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, -0.005))
Mercurial.gamepad1.b.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, 0.005))
* */