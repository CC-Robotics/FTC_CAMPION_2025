package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.distance.cm
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import org.firstinspires.ftc.teamcode.utils.lerp
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

    // in cm, min extend to max extend
    private val rangeOfLength = Pair(44.45, 104.14)
    private const val MAX_POSITION = 400
    val length: Distance
        get() = lerp(rangeOfLength.first, rangeOfLength.second, position.toDouble() / MAX_POSITION).cm

    override fun init(opMode: Wrapper) {
        position = 0
        pidfController.setPIDF(0.01, 0.0, 0.0, 0.025)
        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
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

    fun sBasicallyAt(position: SlidePosition): Boolean {
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