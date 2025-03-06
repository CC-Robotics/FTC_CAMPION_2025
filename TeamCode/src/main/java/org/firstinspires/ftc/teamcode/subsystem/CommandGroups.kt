package org.firstinspires.ftc.teamcode.subsystem

import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.subsystem.HandSubsystem.AXLE_INIT

object CommandGroups {
    fun retract(): Parallel {
        return Parallel(
            LiftSubsystem.retract(),
            HandSubsystem.goTo(HandSubsystem.ServoType.AXLE, AXLE_INIT)
        )
    }

    fun positionForCollection(): Parallel {
        return Parallel(
            LiftSubsystem.goTo(752),
            ArmSubsystem.goTo(485),
            Sequential(
                HandSubsystem.goTo(HandSubsystem.ServoType.WRIST, 0.354),
                HandSubsystem.goTo(HandSubsystem.ServoType.AXLE, 0.459),
                HandSubsystem.updateClawState(HandSubsystem.ClawState.CLOSED)
            )
        )
    }

    fun collect(keybinds: KeybindTemplate): Advancing {
        return Advancing(
            positionForCollection(),
            DrivetrainSubsystem.lockIn(keybinds)
        )
    }
}