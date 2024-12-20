package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.controller.PIDFController
import kotlin.math.max
import kotlin.math.roundToInt

open class PIDFSubsystem : SubsystemCore() {
    open val pidfController = PIDFController(0.00, 0.0, 0.0, 0.0)

    open var position: Int = 0
    open val increment: Int = 5

    open fun adjustPIDFValue(value: String = "p", multiplier: Double = 1.0): Lambda {
        val name = PIDFController.nameMap[value]
        return Lambda("Change $subsystemName PIDF: $name")
            .addRequirements(this::class.java)
            .addExecute {
                PIDFController.adjustPIDF(pidfController, value, multiplier)
            }
    }

    open fun changePosition(telemetry: Telemetry, multiplier: Double = 1.0): Lambda {
        return Lambda("Change $subsystemName position")
            .addRequirements(this::class.java)
            .addExecute {
                position = max(position + increment * multiplier, 0.0).roundToInt()
            }
    }
}