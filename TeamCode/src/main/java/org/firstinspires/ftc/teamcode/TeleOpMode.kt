package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
*
* The drivetrain is currently commented out as the control hub is not attached to the main bot.
* */

@TeleOp(name = "Campion TeleOp 2024-25", group = "Linear OpMode")
abstract class TeleOpMode : LinearOpMode() {
//    private val fL = Motor(hardwareMap, "fL")
//    private val fR = Motor(hardwareMap, "fR")
//    private val bL = Motor(hardwareMap, "bL")
//    private val bR = Motor(hardwareMap, "bR")
    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

//        val drive = MecanumDrive(fL, fR, bL, bR);
        val driverOp = GamepadEx(gamepad1)

        waitForStart()

        while (opModeIsActive()) {
            // Runtime here...
//            drive.driveRobotCentric(
//                driverOp.leftX,
//                driverOp.leftY,
//                driverOp.rightY
//            );
            telemetry.update()
        }
    }
}