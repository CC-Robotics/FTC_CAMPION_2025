package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.utils.round
import java.lang.annotation.Inherited

object ClawSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val wrist by getHardware<Servo>("wrist")
    private val axle by getHardware<Servo>("axle")
    private val claw by getHardware<Servo>("claw")

    private var state = ClawState.OPEN

    override fun init(opMode: Wrapper) {
        wrist.controller.pwmEnable()
        claw.controller.pwmEnable()
        axle.controller.pwmEnable()
//        claw.setPwmEnable()
//        wrist.setPwmEnable()
//        axle.setPwmEnable()
//
//        claw.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
//        wrist.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
//        axle.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        wrist.position = 0.0
        claw.position = 0.0
        axle.position = 0.0
    }

    private fun moveWrist(multiplier: Double, useAsMultiplier: Boolean = true) {
        wrist.position +=(if (useAsMultiplier) 0.001 else 1.0) * multiplier
    }

    private fun moveClaw(multiplier: Double, useAsMultiplier: Boolean = true) {
        claw.position += (if (useAsMultiplier) 0.001 else 1.0) * multiplier
    }

    private fun moveAxel(multiplier: Double, useAsMultiplier: Boolean = true) {
        axle.position += (if (useAsMultiplier) 0.001 else 1.0) * multiplier
    }

    fun moveServoL(which: ServoType, multiplier: Double): Lambda {
        return when (which) {
            ServoType.WRIST -> Lambda("Move Wrist").addRequirements(this::class.java)
                .addExecute { moveWrist(multiplier) }

            ServoType.AXLE -> Lambda("Move Axel").addRequirements(this::class.java)
                .addExecute { moveAxel(multiplier) }

            ServoType.CLAW -> Lambda("Move Claw").addRequirements(this::class.java)
                .addExecute { moveClaw(multiplier) }
        }
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

    override fun periodic(opMode: Wrapper) {
        telemetry.addData("Wrist", wrist.position.round(3))
        telemetry.addData("Claw", claw.position.round(3))
        telemetry.addData("Axle", axle.position.round(3))
    }

    enum class ServoType {
        WRIST,
        AXLE,
        CLAW
    }

    enum class ClawState(val position: Double) {
        OPEN(0.6),
        CLOSED(0.8)
    }
}