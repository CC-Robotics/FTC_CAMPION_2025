package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
*
* The drivetrain is currently commented out as the control hub is not attached to the main bot.
* */

@Autonomous(name = "Red | Auto - N/A | Main", group = "2024-25 OpCodes")
abstract class RedAutoMain : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.update()
        }
    }
}