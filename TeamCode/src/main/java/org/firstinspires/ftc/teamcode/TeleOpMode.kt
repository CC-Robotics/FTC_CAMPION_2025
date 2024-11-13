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
* */

@TeleOp(name = "Campion TeleOp 2024-25", group = "Linear OpMode")
class TeleOpMode : LinearOpMode() {

    // Drivetrain
    private val fL by lazy { Motor(hardwareMap, "fL") }
    private val fR by lazy { Motor(hardwareMap, "fR") }
    private val bL by lazy { Motor(hardwareMap, "bL") }
    private val bR by lazy { Motor(hardwareMap, "bR") }

    // Arm
    private val rightLift by lazy { Motor(hardwareMap, "right_lift") }
    private val leftLift by lazy { Motor(hardwareMap, "left_lift") }
    private val slide by lazy { Motor(hardwareMap, "slide") }

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val drive = MecanumDrive(fL, fR, bL, bR);
        val driverOp = GamepadEx(gamepad1)

        waitForStart()

        while (opModeIsActive()) {
            // Runtime here...
            drive.driveRobotCentric(
                driverOp.leftX,
                driverOp.leftY,
                driverOp.rightY
            );
            telemetry.update()
        }
    }
}