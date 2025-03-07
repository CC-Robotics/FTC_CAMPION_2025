package org.firstinspires.ftc.teamcode.subsystem

import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.KeybindTemplate
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem.AXLE_INIT

object CommandGroups {
    fun retract(): Parallel {
        return Parallel(
            LiftSubsystem.retract(),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.AXLE, AXLE_INIT)
        )
    }

    fun goToCollectPosition(): Parallel {
        return Parallel(
            ArmSubsystem.goTo(302),
            LiftSubsystem.goTo(752),
            GripperSubsystem.setClawState(GripperSubsystem.ClawState.CLOSED),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.WRIST, 0.354),
            GripperSubsystem.goTo(GripperSubsystem.ServoType.AXLE, 0.304)
        )
    }

    fun collect(): Sequential {
        return Sequential(
            goToCollectPosition(),
            DrivetrainSubsystem.lockIn()
        )
    }
}