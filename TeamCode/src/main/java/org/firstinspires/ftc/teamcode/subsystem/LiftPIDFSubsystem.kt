package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited

object LiftPIDFSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val rightLift by getHardware<DcMotorEx>("right_lift")
    private val leftLift by getHardware<DcMotorEx>("left_lift")

    private val PIDFController = PIDFController(0.1, 0.0, 0.0, 0.0)

    private var position: Int = 0
    private const val INCREMENT: Int = 100
    private const val TELEMETRY_KEY = "Lift position"

    override fun periodic(opMode: Wrapper) {
        val leftPower = PIDFController.calculate(leftLift.currentPosition.toDouble(), position.toDouble())
        val rightPower = PIDFController.calculate(rightLift.currentPosition.toDouble(), position.toDouble())

        leftLift.power = leftPower
        rightLift.power = rightPower
    }

    @Suppress("unused")
    fun goUp(telemetry: Telemetry): Lambda {
        return Lambda("Lift go Up")
            .addRequirements(LiftPIDFSubsystem)
            .addExecute {
                position += INCREMENT
                telemetry.addData(TELEMETRY_KEY, position)
                telemetry.update()
            }
    }

    @Suppress("unused")
    fun goDown(telemetry: Telemetry): Lambda {
        return Lambda("Lift go down")
            .addRequirements(LiftPIDFSubsystem)
            .addExecute {
                position -= INCREMENT
                telemetry.addData(TELEMETRY_KEY, position)
                telemetry.update()
            }
    }
}