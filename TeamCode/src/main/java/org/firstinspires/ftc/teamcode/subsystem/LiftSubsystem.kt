package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.utils.applySensitivity
import org.firstinspires.ftc.teamcode.structures.PIDFSubsystem
import java.lang.annotation.Inherited

object LiftSubsystem : PIDFSubsystem() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    override val subsystemName = "Lift"

    private val rightLift by getHardware<DcMotorEx>("right_lift")
    private val leftLift by getHardware<DcMotorEx>("left_lift")

    private val defaultValues = PIDFCoefficients(0.004, 0.02, 0.0, 0.025)
//    private val extendedValues = PIDFValues(0.004, 0.02, 0.0, 0.025)

    private const val MAX_POSITION = 150
//    val rangeOfMotion = Pair(-10, 70)
//    val angle
//        get() = lerp(
//            rangeOfMotion.first,
//            rangeOfMotion.second,
//            position.toDouble() / maxPosition
//        ).deg

    override fun init(opMode: Wrapper) {
        position = 0
        pidfController.setPIDF(defaultValues)

        leftLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightLift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun update(increment: Double) {
        // if you uncomment this, you can't tune anymore LOL
        // pidfController.setPIDF(lerpPIDFValues(defaultValues, extendedValues, 1.0))
        pidfController.setPIDF(Config.LIFT_PIDF)
        changePosition(
            applySensitivity(increment, 1.0, 0.2)
        )
        clampPosition(0, MAX_POSITION)
        val power =
            pidfController.calculate(rightLift.currentPosition.toDouble(), position.toDouble())

        leftLift.power = power
        rightLift.power = power

        telemetry(power)
    }

    fun telemetry(power: Double) {
        telemetry.addData("Left Stick Y", Mercurial.gamepad2.leftStickY.state)
        telemetry.addData("Left Lift Real Position", leftLift.currentPosition)
        telemetry.addData("Right Lift Real Position", rightLift.currentPosition)
        telemetry.addData("$subsystemName Position", position)
        telemetry.addData("Left Lift Power (real vs intended)", "${leftLift.power} vs $power")
        telemetry.addData("Right Lift Power (real vs intended)", "${rightLift.power} vs $power")
    }
}