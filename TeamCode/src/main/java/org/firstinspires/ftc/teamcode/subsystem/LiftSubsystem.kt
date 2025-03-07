package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
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

    private val lift by subsystemCell { getHardware<DcMotorEx>("slide") }

    override val sensitivity = 40

    @JvmField
    var targetPositionTunable = 0

    @JvmField
    var logTelemetry = true

    fun reset() {
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun init(opMode: Wrapper) {
        targetPosition = 0
        targetPositionTunable = 0
        pidfController.setPIDF(0.01, 0.0, 0.0, 0.025)

        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lift.setCurrentAlert(8.4, CurrentUnit.AMPS)
    }

    override fun setTarget(position: Int) {
        targetPositionTunable = position
        targetPosition = position
    }

    fun isAtTarget(): Boolean {
        return basically(lift.currentPosition, targetPosition, sensitivity)
    }

    fun retract() = goTo(0)

//    fun retract() = Lambda("Retract Slide")
//        .addRequirements(LiftSubsystem)
//        .addInit { lift.power = -1.0 }
//        .setFinish { lift.isOverCurrent }
//        .setEnd { interrupted ->
//            lift.power = 0.0
//            if (interrupted) {
//                lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//            }
//        }

    fun goTo(target: Int) = Lambda("Go to $subsystemName position")
        .setExecute { setTarget(target) }
        .setFinish(LiftSubsystem::isAtTarget)

    private fun updatePIDF() {
        pidfController.setPIDF(RobotConfig.LINEAR_SLIDE_PIDF)
        val power =
            pidfController.calculate(lift.currentPosition.toDouble(), targetPosition.toDouble())
        lift.power = power
        Util.telemetry.addData("Setliftpower", power)
    }

    fun update(keybinds: KeybindTemplate) = Lambda("Update Linear Slide")
        .addRequirements(LiftSubsystem)
        .setExecute {
            targetPosition = targetPositionTunable
            incrementPosition(keybinds.slide.state)
            if (RobotConfig.lockLift) {
                updatePIDF()
            } else {
                lift.power = pidfController.f
            }
        }
        .addInterruptible { true }
        .setFinish { false }

    override fun periodic(opMode: Wrapper) {
        if (!logTelemetry) return
        Util.telemetry.addData("current", lift.getCurrent(CurrentUnit.AMPS))
        Util.telemetry.addData("Slide Position", lift.currentPosition)
        Util.telemetry.addData("Slide Target Position", targetPosition)
        Util.telemetry.update()
    }
}