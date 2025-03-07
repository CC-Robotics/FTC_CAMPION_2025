package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.TwoDriverTemplate
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystem.CommandGroups
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem


/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */


@Mercurial.Attach
@DrivetrainSubsystem.Attach
@VisionSubsystem.Attach
@GripperSubsystem.Attach
@ArmSubsystem.Attach
@LiftSubsystem.Attach
open class TeleMain : OpMode() {
    private lateinit var keybinds: KeybindTemplate

    override fun init() {
        keybinds = TwoDriverTemplate(Mercurial.gamepad1, Mercurial.gamepad2)

        ArmSubsystem.defaultCommand = ArmSubsystem.update(keybinds)
        LiftSubsystem.defaultCommand = LiftSubsystem.update(keybinds)
        DrivetrainSubsystem.defaultCommand = DrivetrainSubsystem.driveByGamepad(keybinds)

//        keybinds.axleDown.whileTrue(GripperSubsystem.addTo(GripperSubsystem.ServoType.AXLE, -0.005))
//        keybinds.axleUp.whileTrue(GripperSubsystem.addTo(GripperSubsystem.ServoType.AXLE, 0.005))
//
//        keybinds.wristUp.whileTrue(GripperSubsystem.addTo(GripperSubsystem.ServoType.WRIST, 0.005))
//        keybinds.wristDown.whileTrue(
//            GripperSubsystem.addTo(
//                GripperSubsystem.ServoType.WRIST, -0.005
//            )
//        )

        keybinds.ideallyExtend.onTrue(
            Parallel(
                ArmSubsystem.goTo(190), LiftSubsystem.goTo(3167)
            )
        )

        keybinds.toggleCollection.onTrue(CommandGroups.collect(keybinds))

        keybinds.resetEncoder.onTrue(ArmSubsystem.resetEncoders())
        keybinds.toggleClaw.onTrue(GripperSubsystem.toggleClaw())
        keybinds.retract.onTrue(CommandGroups.retract())
    }

    override fun loop() {
        if (keybinds.axleDown.state) GripperSubsystem.addTo(GripperSubsystem.ServoType.AXLE, -0.005).execute()
        if (keybinds.axleUp.state) GripperSubsystem.addTo(GripperSubsystem.ServoType.AXLE, 0.005).execute()
        if (keybinds.wristUp.state) GripperSubsystem.addTo(GripperSubsystem.ServoType.WRIST, 0.005).execute()
        if (keybinds.wristDown.state) GripperSubsystem.addTo(GripperSubsystem.ServoType.WRIST, -0.005).execute()
        VisionSubsystem.getBestContourAndCache()
        telemetry.update()
    }
}

@TeleOp(name = "Red | Tele - Ri/Xa | Main", group = "2024-25 OpModes")
class RedTeleMain : TeleMain() {
    override fun init() {
        super.init()
        RobotConfig.allianceColour = RobotConfig.SampleColor.RED
    }
}

@TeleOp(name = "Blue | Tele - Ri/Xa | Main", group = "2024-25 OpModes")
class BlueTeleMain : TeleMain() {
    override fun init() {
        super.init()
        RobotConfig.allianceColour = RobotConfig.SampleColor.BLUE
    }
}

/*
Mercurial.gamepad1.x.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, -0.005))
Mercurial.gamepad1.b.whileTrue(ClawSubsystem.moveServoL(ClawSubsystem.ServoType.CLAW, 0.005))
* */