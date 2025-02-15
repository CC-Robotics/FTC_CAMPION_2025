package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import java.lang.annotation.Inherited

object ClawSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val wrist by getHardwareAndCast<Servo, ServoImplEx>("wrist")
    private val claw by getHardwareAndCast<Servo, ServoImplEx>("claw")

    private var state = ClawState.OPEN

    override fun init(opMode: Wrapper) {
        claw.setPwmEnable()
        wrist.setPwmEnable()

        wrist.position = state.position
        claw.position = state.position
    }

    private fun updateClawState(state: ClawState) {
        claw.position = state.position
        this.state = state
        telemetry.addData("Claw", state.name)
    }

    fun updateClawStateL(state: ClawState): Lambda {
        return Lambda("Update Claw State")
            .addRequirements(ClawSubsystem)
            .addExecute {
                updateClawState(state)
            }
    }

    enum class ClawState(val position: Double) {
        OPEN(0.6),
        CLOSED(0.8)
    }
}