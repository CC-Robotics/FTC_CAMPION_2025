package org.firstinspires.ftc.teamcode.subsystem.outdated

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited
import kotlin.math.max

object LiftSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val rightLift by subsystemCell { getHardware<DcMotorEx>("right_lift") }
    private val leftLift by subsystemCell { getHardware<DcMotorEx>("left_lift") }

    private var position: Int = 0
    private const val INCREMENT: Int = 100

    override fun init(opMode: Wrapper) {
        updatePosition()
        rightLift.power = 1.0
        leftLift.power = 1.0
    }

    fun goUp(telemetry: Telemetry): Lambda {
        return Lambda("Lift go Up")
            .addRequirements(LiftSubsystem)
            .addExecute {
                position += INCREMENT
                updatePosition()
                telemetry.addData("Pos", position)
                telemetry.update()
            }
    }

    fun goDown(telemetry: Telemetry): Lambda {
        return Lambda("Lift go Down")
            .addRequirements(LiftSubsystem)
            .addExecute {
                position -= INCREMENT
                position = max(0, position)
                updatePosition()
                telemetry.addData("Pos", position)
                telemetry.update()
            }
    }

    private fun updatePosition() {
        leftLift.targetPosition = position
        rightLift.targetPosition = position
    }
}