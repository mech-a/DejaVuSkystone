package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import java.awt.GridLayout
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JTextField
import javax.swing.SwingConstants

/**
 * Panel for specifying the robot kinematic constraints.
 */
// TODO: specify jerk?
class ConstraintsPanel : JPanel() {
    private class MutableDriveConstraints(
        var maximumVelocity: Double,
        var maximumAcceleration: Double,
        var maximumAngularVelocity: Double,
        var maximumAngularAcceleration: Double
    ) {
        constructor(constraints: DriveConstraints) : this(
            constraints.maxVel,
            constraints.maxAccel,
            constraints.maxAngVel,
            constraints.maxAngAccel
        )

        fun immutable(): DriveConstraints = DriveConstraints(
            maximumVelocity,
            maximumAcceleration,
            0.0,
            maximumAngularVelocity,
            maximumAngularAcceleration,
            0.0
        )
    }

    private val maxVelTextField: JTextField
    private val maxAccelTextField: JTextField
    private val maxAngVelTextField: JTextField
    private val maxAngAccelTextField: JTextField

    private var mutableConstraints: MutableDriveConstraints = MutableDriveConstraints(0.0, 0.0, 0.0, 0.0)

    var onConstraintsUpdateListener: ((DriveConstraints) -> Unit)? = null

    // TODO: make some helpers to make Swing less awful
    init {
        val panel = JPanel()
        panel.layout = GridLayout(0, 2, 5, 5)

        panel.add(JLabel("Max Velocity", SwingConstants.RIGHT))
        maxVelTextField = JTextField()
        maxVelTextField.addChangeListener {
            mutableConstraints.maximumVelocity = maxVelTextField.text.toDoubleOrNull()
                ?: mutableConstraints.maximumVelocity
            fireUpdate()
        }
        panel.add(maxVelTextField)

        panel.add(JLabel("Max Accel", SwingConstants.RIGHT))
        maxAccelTextField = JTextField()
        maxAccelTextField.addChangeListener {
            mutableConstraints.maximumAcceleration = maxAccelTextField.text.toDoubleOrNull()
                ?: mutableConstraints.maximumAcceleration
            fireUpdate()
        }
        panel.add(maxAccelTextField)

        panel.add(JLabel("Max Ang Velocity", SwingConstants.RIGHT))
        maxAngVelTextField = JTextField()
        maxAngVelTextField.addChangeListener {
            mutableConstraints.maximumAngularVelocity = maxAngVelTextField.text.toDoubleOrNull()?.toRadians()
                ?: mutableConstraints.maximumAngularVelocity
            fireUpdate()
        }
        panel.add(maxAngVelTextField)

        panel.add(JLabel("Max Ang Accel", SwingConstants.RIGHT))
        maxAngAccelTextField = JTextField()
        maxAngAccelTextField.addChangeListener {
            mutableConstraints.maximumAngularAcceleration = maxAngAccelTextField.text.toDoubleOrNull()?.toRadians()
                ?: mutableConstraints.maximumAngularAcceleration
            fireUpdate()
        }
        panel.add(maxAngAccelTextField)

        add(panel)
    }

    fun fireUpdate() {
        onConstraintsUpdateListener?.invoke(mutableConstraints.immutable())
    }

    fun updateConstraints(constraints: DriveConstraints) {
        this.mutableConstraints = MutableDriveConstraints(constraints)

        maxVelTextField.text = String.format("%.2f", constraints.maxVel)
        maxAccelTextField.text = String.format("%.2f", constraints.maxAccel)
        maxAngVelTextField.text = String.format("%.2f", constraints.maxAngVel.toDegrees())
        maxAngAccelTextField.text = String.format("%.2f", constraints.maxAngAccel.toDegrees())
    }
}
