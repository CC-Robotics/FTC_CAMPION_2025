package vision

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.RobotConfig
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.tan

class FinalPipeline : OpenCvPipeline() {

    // Camera Intrinsics
    private val fx = 400  // Focal length i

    // 640x480 Camera Matrix
    private val cameraMatrix = Mat(3, 3, CvType.CV_64F).apply {
        put(0, 0, 821.993, 0.0, 330.489) // fx, 0, cx
        put(1, 0, 0.0, 821.993, 248.997) // 0, fy, cy
        put(2, 0, 0.0, 0.0, 1.0)         // 0, 0, 1
    }

    // 640x480 Distortion Coefficients
    private val distCoeffs = Mat(1, 5, CvType.CV_64F).apply {
        put(0, 0, -0.018522, 1.03979, 0.0, 0.0, -3.3171)
    }

    // Initial thresholds (your current fixed values)
    private var blueThreshold = 145.0  // Cb channel
    private var redThreshold = 178.0   // Cr channel
    private var yellowThreshold = 57.0 // Cb channel (inverse)

    // Camera Mounting Angle (in radians!  Positive if tilted *up*)
    private val cameraMountingAngle = 0.0 // Replace with your actual angle

    // Erosion and Dilation elements
    private val erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))
    private val dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))

    // Adaptive thresholding parameters
    private val adaptationRate = 0.1  // How fast thresholds adapt (0-1)
    private val sampleInterval = 10   // Process adaptation every N frames
    private var frameCount = 0

    // Reference values - what we consider "normal" lighting
    private val referenceCbMean = 128.0
    private val referenceCrMean = 128.0

    // Target ranges (useful for debug info)
    private val targetBlueRange = Pair(135.0, 155.0)  // Cb range for blue
    private val targetRedRange = Pair(168.0, 188.0)   // Cr range for red
    private val targetYellowRange = Pair(35.0, 55.0)  // Cb range for yellow (inverse)


    // Working image buffers
    private val ycrcbMat: Mat = Mat()
    private val crMat: Mat = Mat()
    private val cbMat: Mat = Mat()

    private val blueThresholdMat: Mat = Mat()
    private val redThresholdMat: Mat = Mat()
    private val yellowThresholdMat: Mat = Mat()

    private val morphedBlueThreshold: Mat = Mat()
    private val morphedRedThreshold: Mat = Mat()
    private val morphedYellowThreshold: Mat = Mat()

    private var contoursOnPlainImageMat: Mat = Mat()

    // A data class that holds information about a detected contour and has two properties rect and angle
    data class AnalyzedContour(
        val rect: RotatedRect,
        val angle: Double,
        val color: RobotConfig.SampleColor,
        val distance: Double,
        val coords: Pair<Double, Double>,
        val area: Double
    )

    @Config
    companion object {
        @JvmField val analyzedContours =
            mutableListOf<AnalyzedContour>() // This is a mutable list that stores instances of Analyzed Contour
    }

    override fun processFrame(input: Mat): Mat {
        synchronized(analyzedContours) {
            analyzedContours.clear()

            // Convert to YCrCb color space
            Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)

            // Extract the Cb and Cr channels
            Core.extractChannel(ycrcbMat, cbMat, 2) // Cb channel index is 2
            Core.extractChannel(ycrcbMat, crMat, 1) // Cr channel index is 1

            // Periodically update thresholds based on image statistics
            if (frameCount % sampleInterval == 0) {
                adaptThresholds()
            }
            frameCount++

            // Draw debug info about current thresholds
            Imgproc.putText(input, "Blue Cb: ${String.format("%.1f", blueThreshold)}",
                Point(10.0, 30.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.0, 255.0, 255.0), 1)
            Imgproc.putText(input, "Red Cr: ${String.format("%.1f", redThreshold)}",
                Point(10.0, 50.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.0, 255.0, 255.0), 1)
            Imgproc.putText(input, "Yellow Cb: ${String.format("%.1f", yellowThreshold)}",
                Point(10.0, 70.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.0, 255.0, 255.0), 1)

            // Use adaptive thresholds for detection
            findContoursWithAdaptiveThresholds(input)

            // Draw contour count
            Imgproc.putText(input, "Contours: ${analyzedContours.size}",
                Point(10.0, 90.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.0, 255.0, 255.0), 1)
        }
        return input
    }

    private fun adaptThresholds() {
        // Calculate current image statistics
        val cbMean = Core.mean(cbMat).`val`[0]
        val crMean = Core.mean(crMat).`val`[0]

        // Calculate lighting shifts from reference values
        val cbShift = cbMean - referenceCbMean
        val crShift = crMean - referenceCrMean

        // Adjust thresholds based on lighting shifts
        // Blue detection (Cb channel)
        blueThreshold = adjustThreshold(145.0, cbShift)  // 145 is your base threshold

        // Red detection (Cr channel)
        redThreshold = adjustThreshold(173.0, crShift)   // 178 is your base threshold

        // Yellow detection (Cb channel, inverse)
        yellowThreshold = adjustThreshold(57.0, cbShift) // 45 is your base threshold
    }

    private fun adjustThreshold(baseThreshold: Double, shift: Double): Double {
        // Apply a portion of the shift to the threshold
        return baseThreshold + (shift * adaptationRate)
    }

    private fun findContoursWithAdaptiveThresholds(input: Mat) {
        // Apply thresholds with current adaptive values
        Imgproc.threshold(cbMat, blueThresholdMat, blueThreshold, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.threshold(crMat, redThresholdMat, redThreshold, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.threshold(cbMat, yellowThresholdMat, yellowThreshold, 255.0, Imgproc.THRESH_BINARY_INV)


        // Apply morphology to clean up the masks
        morphMask(blueThresholdMat, morphedBlueThreshold)
        morphMask(redThresholdMat, morphedRedThreshold)
        morphMask(yellowThresholdMat, morphedYellowThreshold)

        // Additional morphological operations to close gaps
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 5.0))
        Imgproc.morphologyEx(morphedBlueThreshold, morphedBlueThreshold, Imgproc.MORPH_CLOSE, kernel)
        Imgproc.morphologyEx(morphedRedThreshold, morphedRedThreshold, Imgproc.MORPH_CLOSE, kernel)
        Imgproc.morphologyEx(morphedYellowThreshold, morphedYellowThreshold, Imgproc.MORPH_CLOSE, kernel)

        // Find contours in the masks
        val blueContoursList = ArrayList<MatOfPoint>()
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE)

        val redContoursList = ArrayList<MatOfPoint>()
        Imgproc.findContours(morphedRedThreshold, redContoursList, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE)

        val yellowContoursList = ArrayList<MatOfPoint>()
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE)

        // Create a plain image for drawing contours
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type())

        // Analyze and draw contours
        for (contour in blueContoursList) {
            analyzeContour(contour, input, RobotConfig.SampleColor.BLUE)
        }

        for (contour in redContoursList) {
            analyzeContour(contour, input, RobotConfig.SampleColor.RED)
        }

        for (contour in yellowContoursList) {
            analyzeContour(contour, input, RobotConfig.SampleColor.YELLOW)
        }
    }


    private fun getCoordinates(input: Mat, cX: Int, cY: Int): Pair<Double, Double> {
        val width = input.width().toDouble()
        val height = input.height().toDouble()

        // Calculate center offsets
        val offsetX = cX - (width / 2)
        val offsetY = (height / 2) - cY  // Flipping Y so positive is up

        // Normalize to [-1, 1] range
        val normalizedX = (offsetX / (width / 2)).coerceIn(-1.0, 1.0)
        val normalizedY = (offsetY / (height / 2)).coerceIn(-1.0, 1.0)

        return Pair(normalizedX, normalizedY)
    }

    private fun findContours(input: Mat) {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)

        // Extract the Cb and Cr channels
        Core.extractChannel(ycrcbMat, cbMat, 2) // Cb channel index is 2
        Core.extractChannel(ycrcbMat, crMat, 1) // Cr channel index is 1

        // Threshold the channels to form masks
        Imgproc.threshold(cbMat, blueThresholdMat, 145.0, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.threshold(crMat, redThresholdMat, 178.0, 255.0, Imgproc.THRESH_BINARY)
        Imgproc.threshold(cbMat, yellowThresholdMat, 54.0, 255.0, Imgproc.THRESH_BINARY_INV)

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold)
        morphMask(redThresholdMat, morphedRedThreshold)
        morphMask(yellowThresholdMat, morphedYellowThreshold)

        // Additional morphological operations to close gaps caused by the white line
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 5.0))
        try {
            Imgproc.morphologyEx(
                morphedBlueThreshold,
                morphedBlueThreshold,
                Imgproc.MORPH_CLOSE,
                kernel
            )
            Imgproc.morphologyEx(morphedRedThreshold, morphedRedThreshold, Imgproc.MORPH_CLOSE, kernel)
            Imgproc.morphologyEx(
                morphedYellowThreshold,
                morphedYellowThreshold,
                Imgproc.MORPH_CLOSE,
                kernel
            )

            // Find contours in the masks
            val blueContoursList = ArrayList<MatOfPoint>()
            val blueHierarchy = Mat()
            Imgproc.findContours(
                morphedBlueThreshold,
                blueContoursList,
                blueHierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_NONE
            )

            val redContoursList = ArrayList<MatOfPoint>()
            val redHierarchy = Mat()
            Imgproc.findContours(
                morphedRedThreshold,
                redContoursList,
                redHierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_NONE
            )

            val yellowContoursList = ArrayList<MatOfPoint>()
            val yellowHierarchy = Mat()
            Imgproc.findContours(
                morphedYellowThreshold,
                yellowContoursList,
                yellowHierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_NONE
            )

            // Create a plain image for drawing contours
            input.copyTo(contoursOnPlainImageMat)

            // Analyze and draw contours
            for (contour in blueContoursList) {
                try {
                    analyzeContour(contour, input, RobotConfig.SampleColor.BLUE)
                } finally {
                    contour.release()
                }
            }

            for (contour in redContoursList) {
                try {
                    analyzeContour(contour, input, RobotConfig.SampleColor.RED)
                } finally {
                    contour.release()
                }
            }

            for (contour in yellowContoursList) {
                try {
                    analyzeContour(contour, input, RobotConfig.SampleColor.YELLOW)
                } finally {
                    contour.release()
                }
            }

            // Release hierarchy mats
            blueHierarchy.release()
            redHierarchy.release()
            yellowHierarchy.release()
        } finally {
            // Release the kernel
            kernel.release()
        }
    }

    private fun morphMask(input: Mat, output: Mat) {
        Imgproc.erode(input, output, erodeElement)
        Imgproc.erode(output, output, erodeElement)
        Imgproc.dilate(output, output, dilateElement)
        Imgproc.dilate(output, output, dilateElement)
    }

    private fun getSamplePositions(offsetX: Double, offsetY: Double, distance: Double): Pair<Double, Double> {


        val angleX = atan(offsetX / fx)
        val angleY = atan(offsetY / fx)

        // 5. Calculate Horizontal Offset (Requires Distance)
        val sampleX = distance * tan(angleX)
        val sampleY = distance * tan(angleY)
        return Pair(sampleX, sampleY)

    }

    private fun analyzeContour(contour: MatOfPoint, input: Mat, color: RobotConfig.SampleColor) {
        val points = contour.toArray()
        val contour2f = MatOfPoint2f(*points)

        try {
            val rotatedRectFitToContour = Imgproc.minAreaRect(contour2f)
            val area = Imgproc.contourArea(contour)

            // Area restriction: only process contours within the specified area range
            if (area < 1500 || area > 70000) return

            drawRotatedRect(rotatedRectFitToContour, input, color)
            drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color)

            var rotRectAngle = rotatedRectFitToContour.angle
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90.0
            }

            val angle = -(rotRectAngle - 180)
            drawTagText(rotatedRectFitToContour, "${Math.round(angle)} deg", input, color)

            val moments = Imgproc.moments(contour)
            val m00 = moments.m00
            if (m00 == 0.0) return

            val cX = (moments.m10 / m00).toInt()
            val cY = (moments.m01 / m00).toInt()

            val distanceRaw = estimateDistance(
                Imgproc.minAreaRect(contour2f),
                451.07,
                8.6,
                3.7,
                cameraMatrix,
                distCoeffs
            )

            val distanceCm = distanceRaw * cos(cameraMountingAngle)

            // Calculate Sample Positions
            val (normalizedX, normalizedY) = getCoordinates(input, cX, cY)
            val (sampleX, sampleY) = getSamplePositions(cX.toDouble(), cY.toDouble(), distanceCm)

            Imgproc.putText(
                input,
                "Distance: $distanceCm cm",
                Point(cX.toDouble(), cY.toDouble() + 20),
                Imgproc.FONT_HERSHEY_COMPLEX,
                1.0,
                Scalar(255.0, 255.0, 255.0)
            )

            Imgproc.putText(input, "Norm X: ${sampleX}",
                Point(cX.toDouble(), cY.toDouble() + 40),
                Imgproc.FONT_HERSHEY_COMPLEX, 0.7, Scalar(255.0, 255.0, 255.0))

            Imgproc.putText(input, "Norm Y: ${sampleY}",
                Point(cX.toDouble(), cY.toDouble() + 60),
                Imgproc.FONT_HERSHEY_COMPLEX, 0.7, Scalar(255.0, 255.0, 255.0))

            // Create a temporary Mat for color detection
            val hsvMat = Mat()
            try {
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV)
                val detectedColor = detectColor(hsvMat, cX, cY)

                Imgproc.putText(
                    input,
                    "Color: ${detectedColor}",
                    Point(cX.toDouble(), cY.toDouble() + 80),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    0.7,
                    Scalar(255.0, 255.0, 255.0)
                )

                Imgproc.putText(
                    input,
                    "$area",
                    Point(cX.toDouble(), cY.toDouble() + 100),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    0.7,
                    Scalar(255.0, 255.0, 255.0)
                )

                analyzedContours.add(
                    AnalyzedContour(
                        rotatedRectFitToContour,
                        rotRectAngle,
                        detectedColor,
                        distanceCm,
                        Pair(sampleX, sampleY),
                        area
                    )
                )
            } finally {
                hsvMat.release()
            }
        } finally {
            contour2f.release()
        }
    }

    private fun drawRotatedRect(rect: RotatedRect, mat: Mat, color: RobotConfig.SampleColor) {
        val points = arrayOfNulls<Point>(4)
        rect.points(points)
        val colorScalar = getColorScalar(color)
        for (i in points.indices) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], colorScalar, 2)
        }
    }

    private fun drawTagText(
        rect: RotatedRect,
        text: String,
        mat: Mat,
        color: RobotConfig.SampleColor
    ) {
        val colorScalar = getColorScalar(color)
        Imgproc.putText(
            mat,
            text,
            Point(rect.center.x - 50, rect.center.y + 25),
            Imgproc.FONT_HERSHEY_PLAIN,
            1.0,
            colorScalar,
            1
        )
    }

    private fun getColorScalar(color: RobotConfig.SampleColor): Scalar {
        return when (color) {
            RobotConfig.SampleColor.BLUE -> Scalar(0.0, 0.0, 255.0)
            RobotConfig.SampleColor.YELLOW -> Scalar(255.0, 255.0, 0.0)
            else -> Scalar(255.0, 0.0, 0.0)
        }
    }

    private fun detectColor(hsv: Mat, cX: Int, cY: Int): RobotConfig.SampleColor {
        val pixel = hsv.get(cY, cX)
        val hue = pixel[0]

        return when (hue) {
            in 0.0..10.0, in 160.0..180.0 -> RobotConfig.SampleColor.RED
            in 20.0..30.0 -> RobotConfig.SampleColor.YELLOW
            in 100.0..130.0 -> RobotConfig.SampleColor.BLUE
            else -> RobotConfig.SampleColor.UNKNOWN
        }
    }

    private fun undistortObjectPoints(
        rect: RotatedRect,
        cameraMatrix: Mat,
        distCoeffs: Mat
    ): RotatedRect {
        // Create input object points (center of the rotated rectangle)
        val objectPoints = MatOfPoint2f(Point(rect.center.x, rect.center.y))
        val undistortedPoints = MatOfPoint2f()

        try {
            // Prepare identity rotation and projection matrices
            val R = Mat.eye(3, 3, CvType.CV_64F)
            val P = Mat.eye(3, 4, CvType.CV_64F)

            try {
                // Apply undistortion using the camera matrix and distortion coefficients
                Calib3d.undistortPoints(objectPoints, undistortedPoints, cameraMatrix, distCoeffs, R, P)

                // Retrieve the undistorted point (center) after transformation
                val undistortedCenter = undistortedPoints.toArray()[0]

                // Return a new RotatedRect with the undistorted center, same size, and angle
                return RotatedRect(undistortedCenter, rect.size, rect.angle)
            } finally {
                R.release()
                P.release()
            }
        } finally {
            objectPoints.release()
            undistortedPoints.release()
        }
    }

    fun estimateDistance(
        rect: RotatedRect,
        fEffective: Double,
        realObjectWidth: Double,
        realObjectHeight: Double,
        cameraMatrix: Mat,
        distCoeffs: Mat
    ): Double {
        // Undistort the object first
        val undistortedRect = undistortObjectPoints(rect, cameraMatrix, distCoeffs)

        // Use the corrected object dimensions
        val detectedSizePixels = max(undistortedRect.size.width, undistortedRect.size.height)
        val realObjectSize = max(realObjectWidth, realObjectHeight)

        // Calculate estimated distance
        return (realObjectSize * fEffective) / detectedSizePixels
    }

    fun fetchAnalyzedContours(): List<AnalyzedContour> {
        synchronized(analyzedContours) {
            // Return a copy of the list to prevent concurrent modification issues
            return analyzedContours.toList()
        }
    }

    // Clean up resources when the pipeline is no longer needed
    override fun onViewportTapped() {
        // This method is called when the viewport is tapped
        // Do any cleanup needed here
        // I'm black
    }

    // Add a method to properly release all Mat resources
    fun release() {
        ycrcbMat.release()
        crMat.release()
        cbMat.release()
        blueThresholdMat.release()
        redThresholdMat.release()
        yellowThresholdMat.release()
        morphedBlueThreshold.release()
        morphedRedThreshold.release()
        morphedYellowThreshold.release()
        contoursOnPlainImageMat.release()
        erodeElement.release()
        dilateElement.release()
        cameraMatrix.release()
        distCoeffs.release()
    }
}