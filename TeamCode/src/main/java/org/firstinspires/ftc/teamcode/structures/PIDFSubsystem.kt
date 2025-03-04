package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.util.basically
import org.firstinspires.ftc.teamcode.util.clampInt
import org.firstinspires.ftc.teamcode.controller.PIDFController
import kotlin.math.max
import kotlin.math.roundToInt

open class PIDFSubsystem : SubsystemCore() {
    open val pidfController = PIDFController()

    @JvmField var targetPosition: Int = 0
    open val sensitivity: Int = 5

    open fun incrementPosition(multiplier: Double = 1.0) {
        targetPosition = max(targetPosition + sensitivity * multiplier, 0.0).roundToInt()
    }

    open fun setPosition(position: Int) {
        targetPosition = position
    }

    open fun clampPosition(lo: Int, hi: Int) {
        setPosition(clampInt(targetPosition, lo, hi))
    }

    open fun changePositionL(multiplier: Double = 1.0): Lambda {
        return Lambda("Change $subsystemName position")
            .addRequirements(this::class.java)
            .addExecute {
                incrementPosition(multiplier)
            }
    }
}