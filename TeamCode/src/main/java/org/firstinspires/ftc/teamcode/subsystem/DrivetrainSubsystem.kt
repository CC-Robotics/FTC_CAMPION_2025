package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.controller.PController
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.Util
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.max

@Config
object DrivetrainSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    @JvmField var p = 0.018

    private val pController = PController(p)

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val fR by subsystemCell { getHardware<DcMotorEx>("fR") }
    private val fL by subsystemCell { getHardware<DcMotorEx>("fL") }
    private val bR by subsystemCell { getHardware<DcMotorEx>("bR") }
    private val bL by subsystemCell { getHardware<DcMotorEx>("bL") }

    @JvmField var isLockedIn = false

    fun lockIn(): Lambda {
        return Lambda("Drive to align with goal")
            .setInit {
                isLockedIn = true
                RobotConfig.lockServos = true
            }
            .setFinish { !isLockedIn }
            .setEnd {
                RobotConfig.lockServos = false
            }
    }

    private fun applyPower(x: Double = 0.0, y: Double = 0.0, rx: Double = 0.0): Boolean {
        // Normalize the values so neither exceed +/- 1.0
        val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
        val frontLeftPower = (y + x + rx) / denominator
        val backLeftPower = (y - x + rx) / denominator
        val frontRightPower = (y - x - rx) / denominator
        val backRightPower = (y + x - rx) / denominator

        // Send calculated power to wheels
        fL.power = frontLeftPower
        bL.power = backLeftPower
        fR.power = frontRightPower
        bR.power = backRightPower

        return frontLeftPower + backLeftPower + frontRightPower + backRightPower != 0.0
    }

    fun driveByGamepad(keybinds: KeybindTemplate) = Lambda("Drive with controller")
        .addRequirements(DrivetrainSubsystem)
        .addExecute {
            if (!isLockedIn)
                drive(keybinds.movementX.state, keybinds.movementY.state, keybinds.movementRot.state)
            else {
                val contour = VisionSubsystem.getAnalyzedContours().firstOrNull()
                val driveResult = drive(
                    keybinds.movementX.state,
                    keybinds.movementY.state,
                    keybinds.movementRot.state
                )

                Util.telemetry.addData("locking in", contour)

                if (contour != null) {
                    if (!driveResult) {
                        pController.kp = p
                        Util.telemetry.addData("stuff", "doing")
                        val xPower = pController.calculate(contour.coords.second, -0.5)
                        val yPower = pController.calculate(contour.coords.first, 0.0)
                        drive(xPower, yPower, 0.0)
                        GripperSubsystem.moveWristDegrees(contour.angle).execute()
                        Util.telemetry.addData("discrep x", abs(contour.coords.first - 0.5))
                        Util.telemetry.addData("discrep y", abs(contour.coords.second))
                    } else {
                        Util.telemetry.addLine("ah bwoy")
                    }
                }

                if (VisionSubsystem.isAligned() || keybinds.toggleCollection.state) {
                    isLockedIn = false
                }
            }
        }
        .addInterruptible { true }
        .setFinish { false }

    private fun drive(x: Double, y: Double, rx: Double): Boolean {
        // You may be wondering. Why is rx and y in the wrong place?
        // Idk... it works
        // YOLO
        return applyPower(x * 1.1, -rx, -y)
    }
}