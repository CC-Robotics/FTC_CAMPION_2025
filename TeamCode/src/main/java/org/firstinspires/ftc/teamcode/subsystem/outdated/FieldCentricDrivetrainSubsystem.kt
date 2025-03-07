package org.firstinspires.ftc.teamcode.subsystem.outdated

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.controller.PIDFController
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.basically
import java.lang.annotation.Inherited
import kotlin.math.abs

@Suppress("unused")
@Config
object FieldCentricDrivetrainSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    val fR by subsystemCell { getHardware<DcMotorEx>("fR") }
    val fL by subsystemCell { getHardware<DcMotorEx>("fL") }
    val bR by subsystemCell { getHardware<DcMotorEx>("bR") }
    val bL by subsystemCell { getHardware<DcMotorEx>("bL") }

    private val imu by subsystemCell { getHardware<IMU>("imu") }

    private const val COUNTS_PER_MOTOR_REV = 537.7
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val WHEEL_DIAMETER_INCHES = 3.8
    private const val COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI)
    private const val DRIVE_SPEED = 0.6
    private const val TURN_SPEED = 0.5

    private val pidfController = PIDFController(0.05, 0.0, 0.01, 0.0)

    data class IntendedPosition(var fL: Int, var fR: Int, var bL: Int, var bR: Int)

    private var targetPosition = IntendedPosition(0, 0, 0, 0)

    override fun init(opMode: Wrapper) {
        fR.direction = DcMotorSimple.Direction.REVERSE
        bR.direction = DcMotorSimple.Direction.REVERSE

        fL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        fR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
        )
        imu.initialize(parameters)
    }

    fun move(x: Double, y: Double, theta: Double): Lambda {
        return Lambda("Move bot autonomously")
            .setExecute {
                val wheelBaseX = 18.0 // Distance between left and right wheels in cm
                val wheelBaseY = 34.0 // Distance between front and back wheels in cm
                val wheelRadius = WHEEL_DIAMETER_INCHES * 2.54 / 2 // Convert inches to cm

                val vx = x

                val w1 = (vx - y - (wheelBaseX + wheelBaseY) * theta) / wheelRadius
                val w2 = (vx + y + (wheelBaseX + wheelBaseY) * theta) / wheelRadius
                val w3 = (vx + y - (wheelBaseX + wheelBaseY) * theta) / wheelRadius
                val w4 = (vx - y + (wheelBaseX + wheelBaseY) * theta) / wheelRadius

                targetPosition = IntendedPosition(
                    (w1 * COUNTS_PER_INCH).toInt(),
                    (w2 * COUNTS_PER_INCH).toInt(),
                    (w3 * COUNTS_PER_INCH).toInt(),
                    (w4 * COUNTS_PER_INCH).toInt()
                )

                while (abs(fL.currentPosition - targetPosition.fL) > 10 ||
                    abs(fR.currentPosition - targetPosition.fR) > 10 ||
                    abs(bL.currentPosition - targetPosition.bL) > 10 ||
                    abs(bR.currentPosition - targetPosition.bR) > 10
                ) {

                    val power = pidfController.calculate(
                        (fL.currentPosition + fR.currentPosition + bL.currentPosition + bR.currentPosition) / 4.0
                    )

                    fL.power = power
                    fR.power = power
                    bL.power = power
                    bR.power = power
                }

                fL.power = 0.0
                fR.power = 0.0
                bL.power = 0.0
                bR.power = 0.0
            }
            .addFinish {
                basically(fL.targetPosition.toDouble(), targetPosition.fL.toDouble(), 10.0) &&
                        basically(
                            fR.targetPosition.toDouble(),
                            targetPosition.fR.toDouble(),
                            10.0
                        ) &&
                        basically(
                            bL.targetPosition.toDouble(),
                            targetPosition.bL.toDouble(),
                            10.0
                        ) &&
                        basically(bR.targetPosition.toDouble(), targetPosition.bR.toDouble(), 10.0)
            }
    }

    fun update() {

    }
}
