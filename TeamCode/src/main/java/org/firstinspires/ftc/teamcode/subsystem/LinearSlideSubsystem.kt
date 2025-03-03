package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.distance.cm
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import org.firstinspires.ftc.teamcode.utils.lerp
import java.lang.annotation.Inherited

@com.acmerobotics.dashboard.config.Config
object LinearSlideSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Slide"

    private val slide by subsystemCell { getHardware<DcMotorEx>("slide") }

    private lateinit var dashboardTelemetry: MultipleTelemetry

    // in cm, min extend to max extend
    private val rangeOfLength = Pair(44.45, 104.14)
    private const val MAX_POSITION = 3900
    val length: Distance
        get() = lerp(rangeOfLength.first, rangeOfLength.second, targetPosition.toDouble() / MAX_POSITION).cm

    val extraPIDF = PIDFController(0.0, 0.0, 0.0, 0.0)
    override val increment = 20

    @JvmField var targetPositionTunable = 0

    override fun init(opMode: Wrapper) {
        targetPosition = 0
        targetPositionTunable = 0
        pidfController.setPIDF(0.01, 0.0, 0.0, 0.025)
        dashboardTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun update(increment: Double) {
        pidfController.setPIDF(Config.LINEAR_SLIDE_PIDF)
        targetPosition = targetPositionTunable
        changePosition(increment)
        // clamp
        targetPositionTunable = targetPosition
        val power = pidfController.calculate(slide.currentPosition.toDouble(), targetPosition.toDouble())
        slide.power = power
        telemetry(power)
    }

    fun setPosition(position: SlidePosition) {
        this.targetPosition = position.position
    }

    fun sBasicallyAt(position: SlidePosition): Boolean {
        return isBasicallyAt(position.position)
    }

    fun telemetry(power: Double) {
        dashboardTelemetry.addData("Slide Position", slide.currentPosition)
        dashboardTelemetry.addData("Slide Target Position", targetPosition)
        dashboardTelemetry.update()
        telemetry.addData("$subsystemName Real Position", slide.currentPosition)
        telemetry.addData("$subsystemName Position", targetPosition)
        telemetry.addData("$subsystemName Intended Power vs Power", "$power vs ${slide.power}")
    }

    enum class SlidePosition(val position: Int) {
        VISION_POSITION(300)
    }
}