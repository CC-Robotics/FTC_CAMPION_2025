package org.firstinspires.ftc.teamcode

import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier
import dev.frozenmilk.mercurial.bindings.BoundGamepad

interface KeybindTemplate {
    val togglePIDF: BoundBooleanSupplier

    val movementRot: BoundDoubleSupplier
    val movementX: BoundDoubleSupplier
    val movementY: BoundDoubleSupplier

    val toggleClaw: BoundBooleanSupplier

    val liftUp: BoundBooleanSupplier
    val liftDown: BoundBooleanSupplier

    val slide: BoundDoubleSupplier

    val wristUp: BoundBooleanSupplier
    val wristDown: BoundBooleanSupplier

    val axleUp: BoundBooleanSupplier
    val axleDown: BoundBooleanSupplier
}

class TwoDriverTemplate(gamepad1: BoundGamepad, gamepad2: BoundGamepad) : KeybindTemplate {
    override val togglePIDF = gamepad2.back

    override val movementRot = gamepad1.rightStickX
    override val movementX = gamepad1.leftStickX
    override val movementY = gamepad1.leftStickY

    override val toggleClaw = gamepad2.x

    override val liftUp = gamepad2.dpadUp
    override val liftDown = gamepad2.dpadDown

    override val wristUp = gamepad2.dpadRight
    override val wristDown = gamepad2.dpadLeft

    override val slide = gamepad2.rightStickY

    override val axleUp = gamepad2.y
    override val axleDown = gamepad2.a
}

class OneDriverTemplate(gamepad: BoundGamepad) : KeybindTemplate {
    override val togglePIDF = gamepad.back

    override val movementRot = gamepad.leftStickX
    override val movementX = gamepad.rightStickX
    override val movementY = gamepad.rightStickY

    override val toggleClaw = gamepad.x

    override val axleUp = gamepad.y
    override val axleDown = gamepad.a

    override val liftUp = gamepad.rightBumper
    override val liftDown = gamepad.leftBumper

    override val wristUp = gamepad.dpadUp
    override val wristDown = gamepad.dpadDown

    override val slide = BoundDoubleSupplier {
        gamepad.rightTrigger.state - gamepad.leftTrigger.state
    }
}