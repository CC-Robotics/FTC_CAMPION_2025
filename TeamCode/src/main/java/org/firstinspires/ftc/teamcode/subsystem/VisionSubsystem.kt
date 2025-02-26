package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Config
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import vision.PolishedSampleDetection
import vision.PolishedSampleDetection.AnalyzedContour
import java.lang.annotation.Inherited

object VisionSubsystem : SubsystemCore() {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)

    private lateinit var camera: OpenCvWebcam
    private lateinit var pipeline: PolishedSampleDetection

    val camName by subsystemCell { getHardware<WebcamName>("CrocCam") }

    private fun createCamera(): OpenCvWebcam {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName,

            )

        val camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId)

        return camera
    }

    override fun init(opMode: Wrapper) {
        camera = createCamera()
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

    private fun getAnalyzedContours(min: Int = 0): List<AnalyzedContour> {
        return organizeContours(pipeline.getAnalyzedContours(), Config.allianceColour, min)
    }

    private fun getLargestAnalyzedContour(preExisting: List<AnalyzedContour>?, min: Int = 0): AnalyzedContour? {
        return (preExisting ?: getAnalyzedContours(min)).maxByOrNull { it.area }
    }

    fun getBestContour(): AnalyzedContour? {
        val contours = getAnalyzedContours()
        if (contours.isEmpty()) return null
        return if (contours[0].color == Config.allianceColour) {
            getLargestAnalyzedContour(contours.filter { it.color == Config.allianceColour })
        } else {
            getLargestAnalyzedContour(contours)
        }
    }

    private fun organizeContours(
        contours: List<AnalyzedContour>,
        allianceColor: Config.SampleColor,
        min: Int = 0,
    ): List<AnalyzedContour> {
        return when (allianceColor) {
            Config.SampleColor.RED -> contours.asSequence().filter { redPriority[it.color]!! >= min }
                .sortedByDescending { redPriority[it.color] }.toList()

            Config.SampleColor.BLUE -> contours.asSequence().filter { bluePriority[it.color]!! >= min }
                .sortedByDescending { redPriority[it.color] }.toList()
            else -> emptyList()
        }
    }

    private val redPriority: Map<Config.SampleColor, Int> = mapOf(
        Config.SampleColor.RED to 1,
        Config.SampleColor.YELLOW to 0,
        Config.SampleColor.BLUE to -1,
        Config.SampleColor.UNKNOWN to -1
    )

    private val bluePriority: Map<Config.SampleColor, Int> = mapOf(
        Config.SampleColor.BLUE to 1,
        Config.SampleColor.YELLOW to 0,
        Config.SampleColor.RED to -1,
        Config.SampleColor.UNKNOWN to -1
    )
}