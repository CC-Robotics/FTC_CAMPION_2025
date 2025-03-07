package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.FtcDashboard
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.structures.SubsystemCore
import org.firstinspires.ftc.teamcode.util.Util
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

    var cachedBestContour: AnalyzedContour? = null
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
                FtcDashboard.getInstance().startCameraStream(camera, 0.0);


                // Attach the processing pipeline
                camera.setPipeline(PolishedSampleDetection())
            }

            override fun onError(errorCode: Int) {
                // Handle error (e.g., log the error code)
                Util.telemetry.addData("Camera Error", errorCode)
                Util.telemetry.update()
            }
        })
        pipeline = PolishedSampleDetection()
    }

    override fun periodic(opMode: Wrapper) {
        getBestContourAndCache()
    }

    private fun getAnalyzedContours(min: Int = 0): List<AnalyzedContour> {
        return organizeContours(pipeline.getAnalyzedContours(), RobotConfig.allianceColour, min)
    }

    private fun getLargestAnalyzedContour(preExisting: List<AnalyzedContour>?, min: Int = 0): AnalyzedContour? {
        return (preExisting ?: getAnalyzedContours(min)).maxByOrNull { it.area }
    }

    fun getBestContourAndCache(): AnalyzedContour? {
        val bestContour = getBestContour()
        cachedBestContour = bestContour
        return bestContour
    }

    fun getBestContour(): AnalyzedContour? {
        val contours = getAnalyzedContours()
        if (contours.isEmpty()) return null

        val oppositeColor = if (RobotConfig.allianceColour == RobotConfig.SampleColor.RED) RobotConfig.SampleColor.BLUE else RobotConfig.SampleColor.RED
        val filteredContours = contours.filter { it.color != oppositeColor }
        if (filteredContours.isEmpty()) return null

        return if (filteredContours[0].color == RobotConfig.allianceColour) {
            getLargestAnalyzedContour(filteredContours.filter { it.color == RobotConfig.allianceColour })
        } else {
            getLargestAnalyzedContour(filteredContours)
        }
    }

    private fun organizeContours(
        contours: List<AnalyzedContour>,
        allianceColor: RobotConfig.SampleColor,
        min: Int = 0,
    ): List<AnalyzedContour> {
        return when (allianceColor) {
            RobotConfig.SampleColor.RED -> contours.asSequence().filter { redPriority[it.color]!! >= min }
                .sortedByDescending { redPriority[it.color] }.toList()

            RobotConfig.SampleColor.BLUE -> contours.asSequence().filter { bluePriority[it.color]!! >= min }
                .sortedByDescending { redPriority[it.color] }.toList()
            else -> emptyList()
        }
    }

    private val redPriority: Map<RobotConfig.SampleColor, Int> = mapOf(
        RobotConfig.SampleColor.RED to 1,
        RobotConfig.SampleColor.YELLOW to 0,
        RobotConfig.SampleColor.BLUE to -1,
        RobotConfig.SampleColor.UNKNOWN to -1
    )

    private val bluePriority: Map<RobotConfig.SampleColor, Int> = mapOf(
        RobotConfig.SampleColor.BLUE to 1,
        RobotConfig.SampleColor.YELLOW to 0,
        RobotConfig.SampleColor.RED to -1,
        RobotConfig.SampleColor.UNKNOWN to -1
    )
}