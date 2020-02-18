package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * @param startPose start pose
 */
class PathBuilder private constructor(startPose: Pose2d?, internal val path: Path?, internal val s: Double?) {
    constructor(startPose: Pose2d) : this(startPose, null, null)

    constructor(path: Path, s: Double) : this(null, path, s)

    internal var currentPose: Pose2d? = startPose
    private var currentReversed = false

    private var segments = mutableListOf<PathSegment>()

    /**
     * Reverse the direction of robot travel.
     */
    fun reverse(): PathBuilder {
        currentReversed = !currentReversed
        return this
    }

    /**
     * Sets the robot travel direction.
     */
    fun setReversed(reversed: Boolean): PathBuilder {
        this.currentReversed = reversed
        return this
    }

    /**
     * Adds a line path segment.
     *
     * @param end end position
     * @param interpolator heading interpolator
     */
    @JvmOverloads
    fun lineTo(end: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        val line = if (currentReversed) {
            LineSegment(end, start.vec())
        } else {
            LineSegment(start.vec(), end)
        }

        segments.add(PathSegment(line, interpolator, currentReversed))

        currentPose = Pose2d(end, start.heading)

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param end end position
     */
    fun strafeTo(end: Vector2d) = lineTo(end, ConstantInterpolator(
            if (currentPose == null) {
                path!![s!!].heading
            } else {
                currentPose!!.heading
            }
        ))

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return lineTo(start.vec() + Vector2d(
            distance * cos(start.heading),
            distance * sin(start.heading)
        ))
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): PathBuilder {
        reverse()
        forward(-distance)
        reverse()
        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return strafeTo(start.vec() + Vector2d(
            distance * cos(start.heading + PI / 2),
            distance * sin(start.heading + PI / 2)
        ))
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): PathBuilder {
        return strafeLeft(-distance)
    }

    /**
     * Adds a spline segment.
     *
     * @param end end end
     * @param interpolator heading interpolator
     */
    @JvmOverloads
    fun splineTo(end: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val (startWaypoint, endWaypoint) = if (currentPose == null) {
            val start = path!![s!!].vec()
            val startDeriv = path.internalDeriv(s).vec()
            val startSecondDeriv = path.internalSecondDeriv(s).vec()
            val derivMag = (start distTo end.vec())
            QuinticSpline.Waypoint(start, startDeriv, startSecondDeriv) to
                QuinticSpline.Waypoint(end.vec(), Vector2d(cos(end.heading) * derivMag, sin(end.heading)))
        } else {
            val derivMag = (currentPose!!.vec() distTo end.vec())
            QuinticSpline.Waypoint(currentPose!!.x, currentPose!!.y,
                derivMag * cos(currentPose!!.heading), derivMag * sin(currentPose!!.heading)) to
                QuinticSpline.Waypoint(end.x, end.y,
                derivMag * cos(end.heading), derivMag * sin(end.heading))
        }

        val spline = if (currentReversed) {
            QuinticSpline(endWaypoint, startWaypoint)
        } else {
            QuinticSpline(startWaypoint, endWaypoint)
        }

        segments.add(PathSegment(spline, interpolator, currentReversed))

        currentPose = end

        return this
    }

    /**
     * Constructs the [Path] instance.
     */
    fun build() = Path(segments)
}
