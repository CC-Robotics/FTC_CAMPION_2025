package org.firstinspires.ftc.teamcode.structures

import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell

open class SubsystemCore : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY

    open val subsystemName = this::class.simpleName

    companion object {
        inline fun <reified T> getHardware(id: String): T {
            return FeatureRegistrar.activeOpMode.hardwareMap.get(T::class.java, id)
        }
    }

    override fun preUserInitHook(opMode: Wrapper) {
        init(opMode)
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        periodic(opMode)
    }

    open fun init(opMode: Wrapper) {}

    open fun periodic(opMode: Wrapper) {}
}