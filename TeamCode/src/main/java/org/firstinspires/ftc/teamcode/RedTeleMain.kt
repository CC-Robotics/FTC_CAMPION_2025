package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.structures.PIDFAdjuster
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.FieldCentricDrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftPIDFSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LinearSlidePIDFSubsystem
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import vision.PolishedSampleDetection


/*
* !! WARNING:
* If you were not given an instruction or assigned a responsibility for the bot, please
* do NOT modify any of the code below or for any other files.
* */

@Mercurial.Attach
//@DrivetrainSubsystem.Attach
@FieldCentricDrivetrainSubsystem.Attach
@ClawSubsystem.Attach
@LiftPIDFSubsystem.Attach
// @LinearSlidePIDFSubsystem.Attach
@TeleOp(name = "Red | Tele - N/A | Main", group = "2024-25 OpCodes")
class RedTeleMain : OpMode() {
    private lateinit var pidfAdjuster: PIDFAdjuster
    override fun init() {

        // Obtain the camera monitor view ID for the live viewport
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName,

        )


        // Get the WebcamName for "CrocCam"
        val webcamName = hardwareMap.get(WebcamName::class.java, "CrocCam")


        // Create an instance of OpenCvWebcam with live preview
        val camera: OpenCvCamera =
            OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)


        // Open the camera device asynchronously
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // Start the streaming session with desired resolution and orientation
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)


                // Attach the processing pipeline
                camera.setPipeline(PolishedSampleDetection())
            }

            override fun onError(errorCode: Int) {
                // Handle error (e.g., log the error code)
                telemetry.addData("Camera Error", errorCode)
                telemetry.update()
            }
        })
        val analyzedContours = PolishedSampleDetection().getAnalyzedContours()

        telemetry.addData("Status", "Initialized")

        Mercurial.gamepad1.a.onTrue(ClawSubsystem.open(telemetry))
        Mercurial.gamepad1.b.onTrue(ClawSubsystem.close(telemetry))

        Mercurial.gamepad1.dpadUp.onTrue(LiftPIDFSubsystem.changePositionL())
        Mercurial.gamepad1.dpadUp.onTrue(Lambda("Log").setExecute {
            telemetry.addLine("Dpad Up")
        })
        Mercurial.gamepad1.dpadDown.onTrue(LiftPIDFSubsystem.changePositionL(-1.0))

        Mercurial.gamepad1.dpadLeft.onTrue(LinearSlidePIDFSubsystem.changePositionL())
        Mercurial.gamepad1.dpadRight.onTrue(LinearSlidePIDFSubsystem.changePositionL(-1.0))

        //Mercurial.gamepad1.leftBumper.onTrue(LiftPIDFSubsystem.changeDerivative(telemetry, -1))
        //Mercurial.gamepad1.rightBumper.onTrue(LiftPIDFSubsystem.changeDerivative(telemetry, 1))
        telemetry.addData("Contours found: ", analyzedContours.size)

        pidfAdjuster = PIDFAdjuster(telemetry, Mercurial.gamepad2)
        pidfAdjuster.attach()
        telemetry.update()
    }

    override fun loop() {
        pidfAdjuster.updateTelemetry()
    }
}