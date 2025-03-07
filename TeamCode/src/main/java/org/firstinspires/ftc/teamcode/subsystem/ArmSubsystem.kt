package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import org.firstinspires.ftc.teamcode.util.Util
import org.firstinspires.ftc.teamcode.util.basically
import org.firstinspires.ftc.teamcode.util.proxiedCommand
import org.firstinspires.ftc.teamcode.util.structures.Dual
import org.firstinspires.ftc.teamcode.util.structures.Timer
import java.lang.annotation.Inherited
import com.acmerobotics.dashboard.config.Config as DashboardConfig

@DashboardConfig
object ArmSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val pidfController = PIDFController(useCosineFeedforward = true, ffOffset = -10.0)

    private val right by subsystemCell { getHardware<DcMotorEx>("right_lift") }
    private val left by subsystemCell { getHardware<DcMotorEx>("left_lift") }

    private const val MAX_POSITION = 1250

    private lateinit var dual: Dual

    override val sensitivity = 15

    @JvmField
    var targetPositionTunable = 0
    @JvmField
    var rightFocus = true
    @JvmField
    var logTelemetry = true

    override fun setTarget(position: Int) {
        targetPositionTunable = position
        targetPosition = position
    }

    fun reset() {
        left.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        right.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        left.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        right.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun init(opMode: Wrapper) {
        dual = Dual(left, right)
        pidfController.setPIDF(RobotConfig.LIFT_PIDF)
        setTarget(0)

        right.direction = DcMotorSimple.Direction.REVERSE

        dual.reset()
    }

    private fun updatePIDF() {
        pidfController.setPIDF(RobotConfig.LIFT_PIDF)
        clampPosition(0, MAX_POSITION)
        val motor = if (rightFocus) right else left
        val power =
            pidfController.calculate(motor.currentPosition.toDouble(), targetPosition.toDouble())
        dual.power = power
    }

    fun isAtTarget(): Boolean {
        val motor = if (rightFocus) right else left
        return basically(motor.currentPosition, targetPosition, sensitivity)
    }

    fun goTo(target: Int) = Lambda("Go to $subsystemName position")
        .setExecute { setTarget(target) }
        .setFinish(ArmSubsystem::isAtTarget)

    fun resetEncoders(): Sequential {
        val timer = Timer()
        return Sequential(
            proxiedCommand( Lambda("Gently set down lift")
                .addRequirements(ArmSubsystem)
                .addExecute {
                    if (right.currentPosition > 450) {
                        dual.power = pidfController.f / 3
                        timer.start(right.currentPosition.toLong() - 100)
                    }
                }
                .setFinish { timer.isFinished() }),
            proxiedCommand( Lambda("Settle lift and reset")
                .addRequirements(ArmSubsystem)
                .addExecute {
                    dual.power = 0.0
                    timer.start(200)
                }
                .setFinish { timer.isFinished() }
                .addEnd {
                    dual.reset()
                    setTarget(0)
                })
        )
    }

    fun update(keybinds: KeybindTemplate) = Lambda("Update Arm")
        .addRequirements(ArmSubsystem)
        .setExecute {
            targetPosition = targetPositionTunable
            incrementPosition(keybinds.arm.state)
            if (RobotConfig.lockLift) {
                left.power = pidfController.f
                right.power = pidfController.f
            } else {
                updatePIDF()
            }
        }
        .addInterruptible { true }
        .setFinish { false }

    override fun periodic(opMode: Wrapper) {
        if (!logTelemetry) return
        // Position graph
        Util.telemetry.addData("Left Lift Position", left.currentPosition)
        Util.telemetry.addData("Right Lift Position", right.currentPosition)
        Util.telemetry.addData("Target Position", targetPosition)

        // Power graph
        Util.telemetry.addData("Left Power", left.power)
        Util.telemetry.addData("Right Power", right.power)
    }
}