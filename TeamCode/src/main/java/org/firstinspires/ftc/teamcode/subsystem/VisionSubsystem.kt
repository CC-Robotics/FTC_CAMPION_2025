package org.firstinspires.ftc.teamcode.subsystem

import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import vision.PolishedSampleDetection
import java.lang.annotation.Inherited

object VisionSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private val camera = createCamera()
    lateinit var pipeline: PolishedSampleDetection

    private fun createCamera(): OpenCvWebcam {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName,

            )
        val camName by getHardware<WebcamName>("CrocCam")
        val camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId)

        return camera
    }

    override fun init(opMode: Wrapper) {
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
        pipeline = PolishedSampleDetection()
    }

    fun getAnalyzedContours(): List<PolishedSampleDetection.AnalyzedContour> {
        return pipeline.getAnalyzedContours()
    }

    fun getLargestAnalyzedContour(): PolishedSampleDetection.AnalyzedContour? {
        val contours = pipeline.getAnalyzedContours()
        return contours.maxByOrNull { it.area }
    }
}