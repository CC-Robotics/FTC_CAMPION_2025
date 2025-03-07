package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.Util
import org.firstinspires.ftc.teamcode.util.adjustedDegreeToWristPosition
import org.firstinspires.ftc.teamcode.util.round
import java.lang.annotation.Inherited

@Config
object GripperSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    @JvmField
    var logTelemetry = true

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val wrist by subsystemCell { getHardware<Servo>("wrist") }
    private val axle by subsystemCell { getHardware<Servo>("axle") }
    private val claw by subsystemCell { getHardware<Servo>("claw") }

    private const val WRIST_INIT = 0.515
    const val AXLE_INIT = 0.885

    private var currentClawState = ClawState.CLOSED

    override fun init(opMode: Wrapper) {
        wrist.controller.pwmEnable()
        claw.controller.pwmEnable()
        axle.controller.pwmEnable()

        wrist.position = WRIST_INIT
        claw.position = currentClawState.position
        axle.position = AXLE_INIT
    }

    fun toggleClaw(): Lambda {
        return Lambda("Claw toggle")
            .setInit {
                when (currentClawState) {
                    ClawState.OPEN -> closeClaw().schedule()
                    ClawState.CLOSED -> openClaw().schedule()
                }
            }
            .setFinish { true }
    }

    fun setClawState(state: ClawState): Lambda {
        return Lambda("Set Claw State")
            .setInit {
                when (state) {
                    ClawState.OPEN -> openClaw().schedule()
                    ClawState.CLOSED -> closeClaw().schedule()
                }
            }
            .setFinish { true }
    }

    fun addTo(which: ServoType, amount: Double): Lambda {
        return when (which) {
            ServoType.WRIST -> Lambda("Add to Wrist").addRequirements(GripperSubsystem)
                .addExecute { if (!RobotConfig.lockServos) wrist.position += amount }.setFinish { true }

            ServoType.AXLE -> Lambda("Add to Axle").addRequirements(GripperSubsystem)
                .addExecute { if (!RobotConfig.lockServos) axle.position += amount }.setFinish { true }
        }
    }

    fun goTo(which: ServoType, target: Double): Lambda {
        return when (which) {
            ServoType.WRIST -> Lambda("Move Wrist").addRequirements(GripperSubsystem)
                .addExecute { if (!RobotConfig.lockServos) wrist.position = target }.setFinish { true }

            ServoType.AXLE -> Lambda("Move Axle").addRequirements(GripperSubsystem)
                .addExecute { if (!RobotConfig.lockServos) axle.position = target }.setFinish { true }
        }
    }

    override fun periodic(opMode: Wrapper) {
        if (!logTelemetry) return
        Util.telemetry.addData("Wrist", wrist.position.round(3))
        Util.telemetry.addData("Claw", claw.position.round(3))
        Util.telemetry.addData("Axle", axle.position.round(3))
    }

    private fun closeClaw(): Lambda {
        return Lambda("Close Claw")
            .addRequirements(GripperSubsystem)
            .setInterruptible(false)
            .setInit {
                if (!org.firstinspires.ftc.teamcode.RobotConfig.lockServos) {
                    claw.position = ClawState.CLOSED.position
                    currentClawState = ClawState.CLOSED
                }
            }
    }

    fun moveWristDegrees(deg: Double): Lambda {
        return goTo(ServoType.WRIST, adjustedDegreeToWristPosition(deg))
    }

    private fun openClaw(): Lambda {
        return Lambda("Open Claw")
            .addRequirements(GripperSubsystem)
            .setInterruptible(false)
            .setInit {
                if (!org.firstinspires.ftc.teamcode.RobotConfig.lockServos) {
                    claw.position = ClawState.OPEN.position
                    currentClawState = ClawState.OPEN
                }
            }
    }

    enum class ServoType {
        WRIST,
        AXLE,
    }

    enum class ClawState(val position: Double) {
        OPEN(0.4),
        CLOSED(1.0)
    }
}