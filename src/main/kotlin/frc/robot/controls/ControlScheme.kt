package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Button
import edu.wpi.first.wpilibj2.command.button.Trigger

interface ControlScheme {
    /**
     * the button to zero the encoders
     */
    val zeroEncoders: Button
    /**
     * the button to toggle the brake mode
     */
    val toggleBrakeMode: Button
    /**
     * the button to reset all modules
     */
    val resetAll: Button
    /**
     * the forward value
     */
    val forward: Double
    /**
     * the trigger for the forward value
     */
    val forewardThresholdTrigger: Trigger
    /**
     * the strafe value
     */
    val strafe: Double
    /**
     * the trigger for the strafe value
     */
    val strafeThresholdTrigger: Trigger
    /**
     * the rotation value
     */
    val rotation: Double
    /**
     * the trigger for the rotation value
     */
    val rotationThresholdTrigger: Trigger
}