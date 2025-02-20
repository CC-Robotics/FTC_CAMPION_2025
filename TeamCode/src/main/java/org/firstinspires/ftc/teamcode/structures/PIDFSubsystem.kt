package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.basically
import org.firstinspires.ftc.teamcode.clampInt
import org.firstinspires.ftc.teamcode.controller.PIDFController
import kotlin.math.max
import kotlin.math.roundToInt

open class PIDFSubsystem : SubsystemCore() {
    open val pidfController = PIDFController(0.00, 0.0, 0.0, 0.0)

    open var position: Int = 0
    open val increment: Int = 5

    open fun adjustPIDFValue(value: String = "p", multiplier: Double) {
        PIDFController.adjustPIDF(pidfController, value, multiplier)
    }

    open fun adjustPIDFValueL(value: String = "p", multiplier: Double = 1.0): Lambda {
        val name = PIDFController.nameMap[value]
        return Lambda("Change $subsystemName PIDF: $name")
            .addRequirements(this::class.java)
            .addExecute {
                adjustPIDFValue(value, multiplier)
            }
    }

    open fun changePosition(multiplier: Double = 1.0) {
        position = max(position + increment * multiplier, 0.0).roundToInt()
    }

    open fun clampPosition(lo: Int, hi: Int) {
        position = clampInt(position, lo, hi)
    }

    fun isBasicallyAt(pos: Int): Boolean {
        return basically(position.toDouble(), pos.toDouble(), 5.0)
    }

    open fun changePositionL(multiplier: Double = 1.0): Lambda {
        return Lambda("Change $subsystemName position")
            .addRequirements(this::class.java)
            .addExecute {
                changePosition(multiplier)
            }
    }
}