package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.Config.Behavior.*
import org.firstinspires.ftc.teamcode.structures.PIDFAdjuster
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlideSubsystem
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem


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
    // private lateinit var pidfAdjuster: PIDFAdjuster
    private lateinit var keybindTemplate: KeybindTemplate

    override fun init() {
        telemetry.addData("Status", "Initialized")

        keybindTemplate = TwoDriverTemplate(Mercurial.gamepad1, Mercurial.gamepad2)

        keybindTemplate.togglePIDF.onTrue(Lambda("Toggle PIDF").addExecute {
            Config.usePIDF = !Config.usePIDF
        })

        keybindTemplate.toggleClaw.onTrue(Lambda("Toggle Claw").addExecute {
            when (ClawSubsystem.state) {
                ClawSubsystem.ClawState.OPEN -> ClawSubsystem.updateClawState(ClawSubsystem.ClawState.CLOSED)
                ClawSubsystem.ClawState.CLOSED -> ClawSubsystem.updateClawState(ClawSubsystem.ClawState.OPEN)
            }
        })

//        Mercurial.gamepad2.a.onTrue(Lambda("Change behavior").addExecute {
//            when (Config.behavior) {
//                MANUAL -> Config.behavior = RUN_TO_VISION_POSITION
//                COLLECTING -> Config.behavior = MANUAL
//                RUN_TO_VISION_POSITION -> {}
//            }
//        })

//        Mercurial.gamepad1.x.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, -0.005))
//        Mercurial.gamepad1.b.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, 0.005))

        keybindTemplate.axleDown.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.AXLE, -0.005))
        keybindTemplate.axleUp.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.AXLE, 0.005))

        keybindTemplate.wristUp.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.WRIST, 0.005))
        keybindTemplate.wristDown.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.WRIST, -0.005))

        // pidfAdjuster = PIDFAdjuster.createAndAttach(telemetry, Mercurial.gamepad2)
        telemetry.update()

        var stateIndex = 0
        val states = listOf(LiftSubsystem.LiftState.LOW, LiftSubsystem.LiftState.MIDDLE, LiftSubsystem.LiftState.HIGH)

        keybindTemplate.liftUp.onTrue(Lambda("Lift Up").addExecute {
            stateIndex = (stateIndex + 1) % states.size
            LiftSubsystem.setLiftState(states[stateIndex])
        })

        keybindTemplate.liftDown.onTrue(Lambda("Lift Down").addExecute {
            stateIndex = (stateIndex - 1) % states.size
            LiftSubsystem.setLiftState(states[stateIndex])
        })
    }

    override fun loop() {
        when (Config.behavior) {
            MANUAL -> {
                LiftSubsystem.update()
                LinearSlideSubsystem.update(keybindTemplate.slide.state)
                DrivetrainSubsystem.drive(
                    keybindTemplate.movementX.state,
                    keybindTemplate.movementY.state,
                    keybindTemplate.movementRot.state
                )
            }

            COLLECTING -> {
                // val sample = VisionSubsystem.getBestContour() ?: return
            }

            RUN_TO_VISION_POSITION -> {
                Config.behavior = MANUAL
                // LinearSlideSubsystem.setPosition(SlidePosition.VISION_POSITION)
//                if (LinearSlideSubsystem.isBasicallyAt(SlidePosition.VISION_POSITION)) {
//                    Config.behavior = COLLECTING
//                }
            }
        }
        // pidfAdjuster.updateTelemetry()
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