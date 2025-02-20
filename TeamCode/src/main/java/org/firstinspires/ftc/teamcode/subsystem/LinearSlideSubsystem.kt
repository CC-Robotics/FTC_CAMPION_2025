package org.firstinspires.ftc.teamcode.subsystem

import android.transition.Slide
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import java.lang.annotation.Inherited

object LinearSlideSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Slide"

    private val slide by getHardware<DcMotorEx>("slide")

    val maxValue = 4300
    val maxVal = 1200

    override fun init(opMode: Wrapper) {
        position = 0
        pidfController.setPIDF(0.01, 0.0, 0.0,0.025)
        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.mode = DcMotor.RunMode.RUN_USING_ENCODER
        slide.direction = DcMotorSimple.Direction.REVERSE
    }

    fun update(increment: Double) {
        changePosition(increment)
        val power = pidfController.calculate(slide.currentPosition.toDouble(), position.toDouble())
        slide.power = power
        telemetry(power)
    }

    fun setPosition(position: SlidePosition) {
        this.position = position.position
    }

    fun isBasicallyAt(position: SlidePosition): Boolean {
        return isBasicallyAt(position.position)
    }

    fun telemetry(power: Double) {
        telemetry.addData("$subsystemName Real Position", slide.currentPosition)
        telemetry.addData("$subsystemName Position", position)
        telemetry.addData("$subsystemName Intended Power vs Power", "$power vs ${slide.power}")
    }

    enum class SlidePosition(val position: Int) {
        VISION_POSITION(300)
    }
}