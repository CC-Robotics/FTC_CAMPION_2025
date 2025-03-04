package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.Util
import org.firstinspires.ftc.teamcode.util.round
import java.lang.annotation.Inherited

object HandSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val wrist by subsystemCell { getHardware<Servo>("wrist") }
    private val axle by subsystemCell { getHardware<Servo>("axle") }
    private val claw by subsystemCell { getHardware<Servo>("claw") }

    private var _clawState = ClawState.OPEN
    private var clawState: ClawState
        get() = _clawState
        set(state) {
            claw.position = state.position
            this._clawState = state
            telemetry.addData("Claw", state.name)
        }

    override fun init(opMode: Wrapper) {
        wrist.controller.pwmEnable()
        claw.controller.pwmEnable()
        axle.controller.pwmEnable()

        wrist.position = 0.515
        claw.position = 0.0
        axle.position = 0.885
    }

    fun incrementPosition(which: ServoType, amount: Double): Lambda {
        return when (which) {
            ServoType.WRIST -> Lambda("Move Wrist").addRequirements(this::class.java)
                .addExecute { wrist.position += amount }

            ServoType.AXLE -> Lambda("Move Axel").addRequirements(this::class.java)
                .addExecute {axle.position += amount }

            ServoType.CLAW -> Lambda("Move Claw").addRequirements(this::class.java)
                .addExecute { claw.position += amount }
        }
    }

    fun toggleClaw() = Lambda("Toggle claw")
        .addRequirements(this::class.java)
        .addExecute {
            clawState = when (clawState) {
                ClawState.OPEN -> ClawState.CLOSED
                ClawState.CLOSED -> ClawState.OPEN
            }
        }

//    fun setClawState(state: ClawState): Lambda {
//        return Lambda("Update Claw State")
//            .addRequirements(HandSubsystem)
//            .addExecute {
//                updateClawState(state)
//            }
//    }

    override fun periodic(opMode: Wrapper) {
        Util.telemetry.addData("Wrist", wrist.position.round(3))
        Util.telemetry.addData("Claw", claw.position.round(3))
        Util.telemetry.addData("Axle", axle.position.round(3))
    }

    enum class ServoType {
        WRIST,
        AXLE,
        CLAW
    }

    enum class ClawState(val position: Double) {
        OPEN(0.4),
        CLOSED(1.0)
    }
}