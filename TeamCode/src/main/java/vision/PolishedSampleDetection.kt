package vision

import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.util.lerp
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.tan


class PolishedSampleDetection : OpenCvPipeline() {

    // Camera Intrinsics
    private val fx = 400.0  // Focal length i
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

    // Erosion removes pixels on object boundaries.The basic idea is to erode away the boundaries of the foreground object
    //(The  color filtered object from the color mask). It is useful for removing small white noises, separating two connected objects
    private val erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))
    // Dilation adds pixels to the boundaries of objects in an image. It is useful for joining broken parts of an object, filling small holes, etc.
    private val dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.5, 3.5))
    // A data class that holds information about a detected contour and has two properties rect and angle
    data class AnalyzedContour(val rect: RotatedRect, val angle: Double, val color: RobotConfig.SampleColor, val distance: Double, val coords: Pair<Double, Double>, val area: Double)
    private val analyzedContours = mutableListOf<AnalyzedContour>() // This is a mutable list that stores instances of Analyzed Contour

    override fun processFrame(input: Mat): Mat {
        val hsv = convertToHSV(input)

        // lower boundary RED color range values; Hue (0 - 10)
        val lowerRed1 = Scalar(0.0, 100.0, 20.0)
        val upperRed1 = Scalar(10.0, 255.0, 255.0)
        /*
            Explanation of the Values:
            Hue (H):
            - Red: The range for red is typically between 0-10 and 160-180.
            - Yellow: The range for yellow is typically between 20-30.
            - Blue: The range for blue is typically between 100-130.

            Saturation (S):
            - A minimum value of 100 ensures the colors are vivid enough to be detected.
            - Higher saturation values filter out washed-out colors.

            Value (V):
            - A minimum value of 20 ensures the object is bright enough to be detected.
            - Higher values ensure better detection in varying lighting conditions.
            */

        // Red needs two boundaries due to the fact its on both sides of the hue
        // upper boundary RED color range values; Hue (160 - 180)
        val lowerRed2 = Scalar(160.0, 100.0, 20.0)
        val upperRed2 = Scalar(179.0, 255.0, 255.0)

        // Refined ranges for the color blue
        val lowerBlue1 = Scalar(100.0, 100.0, 63.0)
        val upperBlue1 = Scalar(130.0, 255.0, 255.0)

        // Refined ranges for the color yellow
        val lowerYellow1 = Scalar(20.0, 100.0, 100.0) // Darker yellow
        val upperYellow1 = Scalar(30.0, 255.0, 255.0) // Lighter yellow


        // We then create masks of each color
        val maskRed1 = createColorMask(hsv, lowerRed1, upperRed1)
        val maskRed2 = createColorMask(hsv, lowerRed2, upperRed2)
        val maskRed = combineMasks(maskRed1, maskRed2)

        val maskBlue = createColorMask(hsv, lowerBlue1, upperBlue1)

        val maskYellow = createColorMask(hsv, lowerYellow1, upperYellow1)

        // All filtered color masks are then combined and the erode and dilate elements are then applied
        val combinedMask = combineMasks(maskRed, maskBlue, maskYellow)
        val morphedMask = applyMorphology(combinedMask)
        findAndDrawContours(morphedMask, input)

        // Mats are released to prevent memory leaks
        releaseMats(hsv, maskRed1, maskRed2, maskRed, maskBlue, maskYellow, combinedMask, morphedMask)
        return input
    }

    private fun convertToHSV(input: Mat): Mat {
        val hsv = Mat()
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
        return hsv
    }

    private fun createColorMask(hsv: Mat, lower: Scalar, upper: Scalar): Mat {
        val mask = Mat()
        Core.inRange(hsv, lower, upper, mask)
        return mask
    }

    private fun combineMasks(vararg masks: Mat): Mat {
        val combinedMask = Mat()
        if (masks.isNotEmpty()) {
            masks[0].copyTo(combinedMask)
        }
        for (i in 1 until masks.size) {
            Core.addWeighted(combinedMask, 1.0, masks[i], 1.0, 0.0, combinedMask)
        }
        return combinedMask
    }

    private fun applyMorphology(mask: Mat): Mat {
        val morphedMask = Mat()
        Imgproc.erode(mask, morphedMask, erodeElement)
        Imgproc.erode(morphedMask, morphedMask, erodeElement)
        Imgproc.dilate(morphedMask, morphedMask, dilateElement)
        Imgproc.dilate(morphedMask, morphedMask, dilateElement)
        return morphedMask
    }
    // Calculates the ideal KFactor values
    private fun idealKFactor(distance: Double): Double {
        return when {
            distance <= 30 -> 730000.0 // K factor at 30cm
            distance >= 90 -> 250000.0 // K factor at 90cm
            else -> lerp(730000.0, 250000.0, (distance - 30) / (90 - 30))
        }
    }

    private fun getCoordinates(input: Mat, cX: Int, cY: Int): Pair<Double, Double> {
        val centerX = input.width() / 2.0 // Principal point x-coordinate
        val centerY = input.height() / 2.0 // Principal point y-coordinate
        return Pair(cX - centerX, cY - centerY)
    }
    private fun getSamplePositions(offsetX: Double, offsetY: Double, distance: Double): Pair<Double, Double> {


        val angleX = atan(offsetX / fx)
        val angleY = atan(offsetY / fx)

        // 5. Calculate Horizontal Offset (Requires Distance)
        val sampleX = distance * tan(angleX)
        val sampleY = distance * tan(angleY)
        return Pair(sampleX, sampleY)

    }


    private fun findAndDrawContours(mask: Mat, input: Mat) {
        // List to store all detected contours
        val contours = ArrayList<MatOfPoint>()
        // Find contours in the mask
        Imgproc.findContours(mask, contours, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        // Clear the previous analyzed contours
        analyzedContours.clear()

        // Variable to keep track of the maximum contour area
        var maxArea = -9999999999.0
        // List to store the largest contour
        val bigCnt = ArrayList<MatOfPoint>()
        // Iterate through each contour
        for (contour in contours) {

            // Calculate the area of the contour
            val area = Imgproc.contourArea(contour, false)

            // Skip small contours by setting a minimum area threshold
            if (area < 2500 || area > 50000) continue // Adjusted from 5000 to 1000

            // Calculate the moments of the contour
            val moments = Imgproc.moments(contour)
            val m00 = moments.m00
            if (m00 == 0.0) continue

            // Calculate the rotated rectangle and angle
            val rect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
            val angle = calculateAngle(rect)

            // Calculate the centroid of the contour
            val cX = (moments.m10 / m00).toInt()
            val cY = (moments.m01 / m00).toInt()

            // Check if the current contour has the largest area
            if (area >= maxArea) {
                maxArea = area
                bigCnt.add(contour)

                // Display the centroid coordinates and area on the input image
                Imgproc.putText(input, "$cX $cY $area", Point(cX.toDouble(), cY.toDouble()), Imgproc.FONT_HERSHEY_COMPLEX, 1.0, Scalar(8.0, 232.0, 222.0))

                // 2. Estimate Distance (Using your function)
                val distanceRaw = estimateDistance(rect, 800.0, 8.802, 3.724, cameraMatrix, distCoeffs)

                // 3. Correct Distance for Camera Mounting Angle
                val distanceCm = distanceRaw * cos(cameraMountingAngle)

                // Calculate the distance based on the contour area
                Imgproc.putText(input, "Distance: $distanceCm cm", Point(cX.toDouble(), cY.toDouble() + 20), Imgproc.FONT_HERSHEY_COMPLEX, 1.0, Scalar(255.0, 255.0, 255.0))

                // 1. Calculate Pixel Offset
                val (offsetX, offsetY) = getCoordinates(input, cX, cY)

                // 2. Calculate Sample Positions
                val (sampleX, sampleY) = getSamplePositions(offsetX, offsetY, distanceCm)

                // Display the horizontal offset
                Imgproc.putText(
                    input,
                    "H Offset: $sampleX cm",
                    Point(cX.toDouble(), cY.toDouble() + 40), // Adjust position as needed
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    0.7,
                    Scalar(255.0, 255.0, 255.0)
                )

                val color = detectColor(input, cX, cY)


                analyzedContours.add(
                    AnalyzedContour(
                        rect,
                        angle,
                        color,
                        distanceCm,
                        Pair(sampleX, sampleY),
                        rect.size.area()
                    ))
                // Add the analyzed contour to the list
                drawRotatedRect(rect, angle, input)
            }
        }

        // Draw only the contours that passed the area filter
        Imgproc.drawContours(input, bigCnt, -1, Scalar(0.0, 255.0, 0.0), 2)
    }
    // Calculates the distance to the object based on its area
    private fun calculateDistance(area: Double): Double {

        // Assuming a linear relationship for simplicity:
        // Distance (cm) = k / Area
        // where k is a constant determined by calibration.

        val k = 464015 // This value should be determined through calibration.

        return k / area // Returns distance in centimeters
    }

    // Calculates the angle of the rotated rectangle
    private fun calculateAngle(rect: RotatedRect): Double {
        // Get the angle of the rotated rectangle
        var rotRectAngle = rect.angle
        // Adjust the angle based on the rectangle dimensions
        if (rect.size.width < rect.size.height) {
            rotRectAngle += 90.0
        }
        // Calculate the final angle
        val angle = -(rotRectAngle - 180)
        return angle
    }

    private fun undistortObjectPoints(rect: RotatedRect, cameraMatrix: Mat, distCoeffs: Mat): RotatedRect {
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



    fun estimateDistance(rect: RotatedRect, fEffective: Double, realObjectWidth: Double, realObjectHeight: Double, cameraMatrix: Mat, distCoeffs: Mat): Double {
        // Undistort the object first
        val undistortedRect = undistortObjectPoints(rect, cameraMatrix, distCoeffs)

        // Use the corrected object dimensions
        val detectedSizePixels = max(undistortedRect.size.width, undistortedRect.size.height)
        val realObjectSize = max(realObjectWidth, realObjectHeight)

        // Calculate estimated distance
        return (realObjectSize * fEffective) / detectedSizePixels
    }



    // Draws the rotated rectangle and its angle on the image
    private fun drawRotatedRect(rect: RotatedRect, angle: Double, mat: Mat) {
        val textPosition = Point(rect.center.x - 50, rect.center.y + 25)
        Imgproc.putText(
            mat,
            "${Math.round(angle)} deg",
            textPosition,
            Imgproc.FONT_HERSHEY_PLAIN,
            1.0,
            Scalar(0.0, 255.0, 0.0),
            1
        )
        val points = arrayOfNulls<Point>(4)
        rect.points(points)
        for (i in points.indices) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], Scalar(0.0, 255.0, 0.0), 2)
        }
    }

    private fun releaseMats(vararg mats: Mat) {
        for (mat in mats) {
            mat.release()
        }
    }


    private fun detectColor(input: Mat, cX: Int, cY: Int): RobotConfig.SampleColor {
        val hsv = Mat()
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
        val pixel = hsv.get(cY, cX)
        val hue = pixel[0]
        hsv.release()

        return when (hue) {
            in 0.0..10.0, in 160.0..180.0 -> RobotConfig.SampleColor.RED
            in 20.0..30.0 -> RobotConfig.SampleColor.YELLOW
            in 100.0..130.0 -> RobotConfig.SampleColor.BLUE
            else -> RobotConfig.SampleColor.UNKNOWN
        }
    }


    fun getAnalyzedContours(): List<AnalyzedContour> {
        return analyzedContours
    }
}