package vision

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

    // Camera Mounting Angle (in radians!  Positive if tilted *up*)
    private val cameraMountingAngle = 0.0 // Replace with your actual angle

    // Erosion and Dilation elements
    private val erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))
    private val dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))

    // Working image buffers
    private var ycrcbMat: Mat = Mat()
    private var crMat: Mat = Mat()
    private var cbMat: Mat = Mat()

    private var blueThresholdMat: Mat = Mat()
    private var redThresholdMat: Mat = Mat()
    private var yellowThresholdMat: Mat = Mat()

    private var morphedBlueThreshold: Mat = Mat()
    private var morphedRedThreshold: Mat = Mat()
    private var morphedYellowThreshold: Mat = Mat()

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

    @Volatile
    private var analyzedContours =
        mutableListOf<AnalyzedContour>() // This is a mutable list that stores instances of Analyzed Contour

    override fun processFrame(input: Mat): Mat {
        findContours(input)
        return input
    }

    private fun getCoordinates(input: Mat, cX: Int, cY: Int): Pair<Double, Double> {
        val centerX = input.width() / 2.0
        val centerY = input.height() / 2.0
        return Pair(cX - centerX, cY - centerY)
    }

    fun getSamplePositions(
        offsetX: Double,
        offsetY: Double,
        distance: Double
    ): Pair<Double, Double> {


        val angleX = atan(offsetX / fx)
        val angleY = atan(offsetY / fx)

        // 5. Calculate Horizontal Offset (Requires Distance)
        val sampleX = distance * tan(angleX)
        val sampleY = distance * tan(angleY)
        return Pair(sampleX, sampleY)

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
        Imgproc.findContours(
            morphedBlueThreshold,
            blueContoursList,
            Mat(),
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_NONE
        )

        val redContoursList = ArrayList<MatOfPoint>()
        Imgproc.findContours(
            morphedRedThreshold,
            redContoursList,
            Mat(),
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_NONE
        )

        val yellowContoursList = ArrayList<MatOfPoint>()
        Imgproc.findContours(
            morphedYellowThreshold,
            yellowContoursList,
            Mat(),
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_NONE
        )

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

        // Release Mats to prevent memory leaks
        releaseMats(
            ycrcbMat,
            crMat,
            cbMat,
            blueThresholdMat,
            redThresholdMat,
            yellowThresholdMat,
            morphedBlueThreshold,
            morphedRedThreshold,
            morphedYellowThreshold,
            contoursOnPlainImageMat
        )
    }

    private fun morphMask(input: Mat, output: Mat) {
        Imgproc.erode(input, output, erodeElement)
        Imgproc.erode(output, output, erodeElement)
        Imgproc.dilate(output, output, dilateElement)
        Imgproc.dilate(output, output, dilateElement)
    }

    private fun analyzeContour(contour: MatOfPoint, input: Mat, color: RobotConfig.SampleColor) {
        val points = contour.toArray()
        val contour2f = MatOfPoint2f(*points)

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

        // 1. Calculate Pixel Offset
        val (offsetX, offsetY) = getCoordinates(input, cX, cY)

        // 2. Calculate Sample Positions
        val (sampleX, sampleY) = getSamplePositions(offsetX, offsetY, distanceCm)

        Imgproc.putText(
            input,
            "Distance: $distanceCm cm",
            Point(cX.toDouble(), cY.toDouble() + 20),
            Imgproc.FONT_HERSHEY_COMPLEX,
            1.0,
            Scalar(255.0, 255.0, 255.0)
        )
        Imgproc.putText(
            input,
            "H Offset: $sampleX cm",
            Point(cX.toDouble(), cY.toDouble() + 40),
            Imgproc.FONT_HERSHEY_COMPLEX,
            0.7,
            Scalar(255.0, 255.0, 255.0)
        )

        val detectedColor = detectColor(input, cX, cY)
        Imgproc.putText(
            input,
            "Color: ${detectedColor} cm",
            Point(cX.toDouble(), cY.toDouble() + 60),
            Imgproc.FONT_HERSHEY_COMPLEX,
            0.7,
            Scalar(255.0, 255.0, 255.0)
        )

        Imgproc.putText(
            input,
            "$area",
            Point(cX.toDouble(), cY.toDouble() + 10),
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


    private fun detectColor(input: Mat, cX: Int, cY: Int): RobotConfig.SampleColor {
        val hsv = Mat()
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
        val pixel = hsv.get(cY, cX)
        val hue = pixel[0]
        val saturation = pixel[1]
        val value = pixel[2]

        return when (hue) {
            in 0.0..10.0, in 160.0..180.0 -> RobotConfig.SampleColor.RED
            in 20.0..30.0 -> RobotConfig.SampleColor.YELLOW
            in 100.0..130.0 -> RobotConfig.SampleColor.BLUE
            else -> RobotConfig.SampleColor.UNKNOWN
        }
    }

    private fun releaseMats(vararg mats: Mat) {
        for (mat in mats) {
            mat.release()
        }
    }

    private fun undistortObjectPoints(
        rect: RotatedRect,
        cameraMatrix: Mat,
        distCoeffs: Mat
    ): RotatedRect {
        // Create input object points (center of the rotated rectangle)
        val objectPoints = MatOfPoint2f(Point(rect.center.x, rect.center.y))

        // Prepare Mat for undistorted points
        val undistortedPoints = MatOfPoint2f()

        // Prepare identity rotation and projection matrices (you can use them if needed)
        val R = Mat.eye(3, 3, CvType.CV_64F)  // Identity rotation matrix
        val P = Mat.eye(3, 4, CvType.CV_64F)  // Identity projection matrix

        // Apply undistortion using the camera matrix and distortion coefficients
        Calib3d.undistortPoints(objectPoints, undistortedPoints, cameraMatrix, distCoeffs)

        // Retrieve the undistorted point (center) after transformation
        val undistortedCenter = undistortedPoints.toArray()[0]

        // Return a new RotatedRect with the undistorted center, same size, and angle
        return RotatedRect(undistortedCenter, rect.size, rect.angle)
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

    @Synchronized
    fun getAnalyzedContours(): List<AnalyzedContour> {
        return analyzedContours
    }
}
