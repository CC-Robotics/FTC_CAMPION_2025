package org.firstinspires.ftc.teamcode.structures

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.controller.PIDFController

open class PIDFSubsystem : SubsystemCore() {
    open val pidfController = PIDFController(0.00, 0.0, 0.0, 0.0)

    open var position: Int = 0
    open val increment: Int = 100

    open fun applyPIDF(motor: DcMotorEx) {
        val power = pidfController.calculate(motor.currentPosition.toDouble(), position.toDouble())
        motor.power = power
    }

    open fun adjustPIDFValue(value: String = "p", multiplier: Int = 1): Lambda {
        val name = PIDFController.nameMap[value]
        return Lambda("Change $subsystemName PIDF: $name")
            .addRequirements(this::class.java)
            .addExecute {
                PIDFController.adjustPIDF(pidfController, value, PIDFController.DEFAULT_INCREMENT * multiplier)
            }
    }

    open fun changePosition(telemetry: Telemetry, multiplier: Int = 1): Lambda {
        return Lambda("Change $subsystemName position")
            .addRequirements(this::class.java)
            .addExecute {
                position += (increment * multiplier)
            }
    }
}