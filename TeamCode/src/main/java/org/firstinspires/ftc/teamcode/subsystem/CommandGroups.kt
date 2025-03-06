package org.firstinspires.ftc.teamcode.subsystem

import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.subsystem.HandSubsystem.AXLE_INIT

object CommandGroups {
    fun retract(): Parallel {
        return Parallel(
            LiftSubsystem.goTo(0),
            HandSubsystem.goTo(HandSubsystem.ServoType.AXLE, AXLE_INIT)
        )
    }
    private fun positionForCollection(): Command {
        return Sequential(
            ArmSubsystem.goTo(437),
            LiftSubsystem.goTo(752),
            Sequential(
                HandSubsystem.goTo(HandSubsystem.ServoType.WRIST, 0.515),
                HandSubsystem.goTo(HandSubsystem.ServoType.AXLE, 0.885),
                HandSubsystem.updateClawState(HandSubsystem.ClawState.CLOSED)
            ),
        )
    }

    fun collect(keybinds: KeybindTemplate): Sequential {
        return Sequential(
            positionForCollection(),
            DrivetrainSubsystem.lockIn(keybinds)
        )
    }
}