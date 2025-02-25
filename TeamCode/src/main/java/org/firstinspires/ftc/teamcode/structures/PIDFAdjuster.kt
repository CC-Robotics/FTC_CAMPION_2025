package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlideSubsystem

class PIDFAdjuster(private val telemetry: Telemetry, private val gamepad: BoundGamepad) {
    private var enabled = false

    private var currentController = 0
    private val controllers =
        listOf(LiftSubsystem.pidfController, LinearSlideSubsystem.pidfController)
    private val names = listOf("Lift", "Linear Slide", "Collection")

    private var currentModifier = 0
    private val modifiers = listOf("p", "i", "d", "f")

    companion object {
        fun createAndAttach(telemetry: Telemetry, gamepad: BoundGamepad): PIDFAdjuster {
            val adjuster = PIDFAdjuster(telemetry, gamepad)
            adjuster.attach()
            return adjuster
        }
    }

    fun updateTelemetry() {

        telemetry.addData(
            "PIDF Adjuster", if (enabled) "On (${names[currentController]} | ${
                PIDFController.nameMap[modifiers[currentModifier]]
            })" else "Off"
        )
        telemetry.addData(
            "PIDF Adjuster Value",
            "(${
                PIDFController.getPIDFValue(
                    controllers[currentController],
                    modifiers[currentModifier]
                )
            })"
        )
        telemetry.update()
    }

    fun attach() {

        gamepad.a.onTrue(Lambda("PIDF Adjuster Toggle").addExecute {
            enabled = !enabled
            updateTelemetry()
        })

        gamepad.dpadUp.onTrue(Lambda("PIDF Adjuster Up").addExecute {
            if (enabled) {
                currentController = (currentController + 1) % controllers.size
                updateTelemetry()
            }
        })

        gamepad.dpadDown.onTrue(Lambda("PIDF Adjuster Down").addExecute {
            if (enabled) {
                currentController = (currentController - 1 + controllers.size) % controllers.size
                updateTelemetry()
            }
        })

        gamepad.dpadLeft.onTrue(Lambda("PIDF Adjuster Left").addExecute {
            if (enabled) {
                currentModifier = (currentModifier - 1 + modifiers.size) % modifiers.size
                updateTelemetry()
            }
        })

        gamepad.dpadRight.onTrue(Lambda("PIDF Adjuster Right").addExecute {
            if (enabled) {
                currentModifier = (currentModifier + 1) % modifiers.size
                updateTelemetry()
            }
        })

        gamepad.leftBumper.onTrue(Lambda("PIDF Adjuster Decrease").addExecute {
            if (enabled) {
                PIDFController.adjustPIDF(
                    controllers[currentController],
                    modifiers[currentModifier],
                    -1.0
                )
                updateTelemetry()
            }
        })

        gamepad.rightBumper.onTrue(Lambda("PIDF Adjuster Increase").addExecute {
            if (enabled) {
                PIDFController.adjustPIDF(
                    controllers[currentController],
                    modifiers[currentModifier],
                    1.0
                )
                updateTelemetry()
            }
        })
    }
}