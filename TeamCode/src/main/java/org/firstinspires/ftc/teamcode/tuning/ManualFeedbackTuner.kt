package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer

class ManualFeedbackTuner : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

                if (drive.localizer is TwoDeadWheelLocalizer) {
                    if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0.0) {
                        throw RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                    }
                } else if (drive.localizer is ThreeDeadWheelLocalizer) {
                    if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0.0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1.0) {
                        throw RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                    }
                }
                waitForStart()

                while (opModeIsActive()) {
                    runBlocking(
                        drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                            .lineToX(DISTANCE)
                            .lineToX(0.0)
                            .build()
                    )
                }
            }
            else -> {
                throw RuntimeException()
            }
        }
    }

    companion object {
        var DISTANCE: Double = 64.0
    }
}
