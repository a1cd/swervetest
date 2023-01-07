package frc.robot.controls

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.Button

/**
 * A control scheme is a set of buttons and triggers that are used to control the robot.
 */
class DefaultControlScheme(
    var xbox: XboxController = XboxController(0)
): ControlScheme {
    override val forward: Double
        get() = // random noise
            xbox.leftY + (Math.random() - 0.5) * 0.02

    override val strafe: Double
        get() = xbox.leftX + (Math.random() - 0.5) * 0.02

    override val rotation: Double
        get() = xbox.rightX + (Math.random() - 0.5) * 0.02

    override val zeroEncoders: Button
        get() = Button { xbox.aButton }
    override val toggleBrakeMode: Button
        get() = Button { xbox.bButton }
    override val resetAll: Button
        get() = Button { xbox.xButton }
}
