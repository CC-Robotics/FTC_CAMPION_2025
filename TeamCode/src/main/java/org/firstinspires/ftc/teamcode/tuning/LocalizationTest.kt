package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Drawing.drawRobot
import org.firstinspires.ftc.teamcode.MecanumDrive

class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

                waitForStart()

                while (opModeIsActive()) {
                    drive.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                -gamepad1.left_stick_y.toDouble(),
                                -gamepad1.left_stick_x.toDouble()
                            ),
                            -gamepad1.right_stick_x.toDouble()
                        )
                    )

                    drive.updatePoseEstimate()

                    val pose = drive.localizer.pose
                    telemetry.addData("x", pose!!.position.x)
                    telemetry.addData("y", pose.position.y)
                    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()))
                    telemetry.update()

                    val packet = TelemetryPacket()
                    packet.fieldOverlay().setStroke("#3F51B5")
                    drawRobot(packet.fieldOverlay(), pose)
                    FtcDashboard.getInstance().sendTelemetryPacket(packet)
                }
            }
            else -> {
                throw RuntimeException()
            }
        }
    }
}
