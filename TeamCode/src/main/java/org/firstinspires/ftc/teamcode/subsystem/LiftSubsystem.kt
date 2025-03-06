package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.Util
import org.firstinspires.ftc.teamcode.util.basically
import java.lang.annotation.Inherited

@com.acmerobotics.dashboard.config.Config
object LiftSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Slide"

    private val slide by subsystemCell { getHardware<DcMotorEx>("slide") }

    override val sensitivity = 40

    @JvmField
    var targetPositionTunable = 0

    override fun init(opMode: Wrapper) {
        targetPosition = 0
        targetPositionTunable = 0
        pidfController.setPIDF(0.01, 0.0, 0.0, 0.025)

        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slide.setCurrentAlert(7.5, CurrentUnit.AMPS)
    }

    override fun setTarget(position: Int) {
        targetPosition = position
        targetPositionTunable = position
    }

    fun isAtTarget(): Boolean {
        return basically(slide.currentPosition, targetPosition, sensitivity)
    }

    fun retract() = Lambda("Retract Slide")
        .addRequirements(LiftSubsystem)
        .addInit { slide.power = -1.0 }
        .setFinish { slide.isOverCurrent }
        .setEnd { interrupted ->
            slide.power = 0.0
            if (interrupted) {
                slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                slide.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }

    fun goTo(target: Int) = Lambda("Go to $subsystemName position")
            .setExecute { setTarget(target) }
            .setFinish(LiftSubsystem::isAtTarget)

    private fun updatePIDF() {
        pidfController.setPIDF(Config.LINEAR_SLIDE_PIDF)
        val power =
            pidfController.calculate(slide.currentPosition.toDouble(), targetPosition.toDouble())
        slide.power = power
    }

    fun update(keybinds: KeybindTemplate) = Lambda("Update Linear Slide")
        .addRequirements(LiftSubsystem)
        .setExecute {
            incrementPosition(keybinds.slide.state)
            updatePIDF()
            log()
        }
        .addInterruptible { true }
        .setFinish { false }

    private fun log() {
        Util.telemetry.addData("Slide Position", slide.currentPosition)
        Util.telemetry.addData("Slide Target Position", targetPosition)
        Util.telemetry.update()
    }

    enum class SlidePosition(val position: Int) {
        VISION_POSITION(300)
    }
}

/*
*     // in cm, min extend to max extend
    private val rangeOfLength = Pair(44.45, 104.14)
    private const val MAX_POSITION = 3900

    val length: Distance
        get() = lerp(rangeOfLength.first, rangeOfLength.second, targetPosition.toDouble() / MAX_POSITION).cm
* */