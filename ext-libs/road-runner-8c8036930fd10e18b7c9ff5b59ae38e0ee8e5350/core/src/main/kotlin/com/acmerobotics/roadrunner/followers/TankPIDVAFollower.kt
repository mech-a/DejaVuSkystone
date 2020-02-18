package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, one feedback loop controls the path displacement (that is, x in the robot reference frame), and
 * another feedback loop to minimize cross track (lateral) error via heading correction (overall, very similar to
 * [HolonomicPIDVAFollower] except adjusted for the nonholonomic constraint). Feedforward is applied at the wheel level.
 *
 * @param axialCoeffs PID coefficients for the robot axial (robot X) controller
 * @param crossTrackCoeffs PID coefficients for the robot heading controller based on cross track error
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
class TankPIDVAFollower @JvmOverloads constructor(
    axialCoeffs: PIDCoefficients,
    crossTrackCoeffs: PIDCoefficients,
    admissibleError: Pose2d = Pose2d(),
    timeout: Double = 0.0,
    clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val axialController = PIDFController(axialCoeffs)
    private val crossTrackController = PIDFController(crossTrackCoeffs)

    override var lastError: Pose2d = Pose2d()

    override fun internalUpdate(currentPose: Pose2d): DriveSignal {
        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetVel = trajectory.velocity(t)
        val targetAccel = trajectory.acceleration(t)

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val poseError = Kinematics.calculatePoseError(targetPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and position = 0
        axialController.targetPosition = poseError.x
        crossTrackController.targetPosition = poseError.y

        // note: feedforward is processed at the wheel level; velocity is only passed here to adjust the derivative term
        val axialCorrection = axialController.update(0.0, targetRobotVel.x)
        val headingCorrection = crossTrackController.update(0.0, targetRobotVel.y)

        val correctedVelocity = targetRobotVel + Pose2d(
            axialCorrection,
            0.0,
            headingCorrection
        )

        lastError = poseError

        return DriveSignal(correctedVelocity, targetRobotAccel)
    }
}
