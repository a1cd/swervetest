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
        get() = Trigger { (forward > 0.05) || (forward < -0.05) }
    /**
     * the strafe value
     */
    val strafe: Double
    /**
     * the trigger for the strafe value
     */
    val strafeThresholdTrigger: Trigger
        get() = Trigger { (strafe > 0.05) || (strafe < -0.05) }
    /**
     * the rotation value
     */
    val rotation: Double
    /**
     * the trigger for the rotation value
     */
    val rotationThresholdTrigger: Trigger
        get() = Trigger { (rotation > 0.05) || (rotation < -0.05) }
}