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

object LinearSlidePIDFSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val slide by getHardware<DcMotorEx>("slide")

    private val PIDFController = PIDFController(0.1, 0.0, 0.0, 0.0)

    private var position: Int = 0
    private const val INCREMENT: Int = 100
    private const val TELEMETRY_KEY = "Linear slide position"

    override fun periodic(opMode: Wrapper) {
        val slidePower = PIDFController.calculate(slide.currentPosition.toDouble(), position.toDouble())
        slide.power = slidePower
    }

    @Suppress("unused")
    fun goUp(telemetry: Telemetry): Lambda {
        return Lambda("Linear slide up")
            .addRequirements(LinearSlidePIDFSubsystem)
            .addExecute {
                position += INCREMENT
                telemetry.addData(TELEMETRY_KEY, position)
                telemetry.update()
            }
    }

    @Suppress("unused")
    fun goDown(telemetry: Telemetry): Lambda {
        return Lambda("Linear slide down")
            .addRequirements(LinearSlidePIDFSubsystem)
            .addExecute {
                position -= INCREMENT
                telemetry.addData(TELEMETRY_KEY, position)
                telemetry.update()
            }
    }
}