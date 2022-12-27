package frc.robot.controls

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.Button
import edu.wpi.first.wpilibj2.command.button.Trigger

class DefaultControlScheme(
    var xbox: XboxController = XboxController(0)
): ControlScheme {
    override val forward: Double
        get() = xbox.leftY

    override val forewardThresholdTrigger: Trigger
        get() = Trigger { (xbox.leftY > 0.05) || (xbox.leftY < -0.05) }

    override val strafe: Double
        get() = xbox.leftX

    override val strafeThresholdTrigger: Trigger
        get() = Trigger { (xbox.leftX > 0.05) || (xbox.leftX < -0.05) }
    override val rotation: Double
        get() = xbox.rightX

    override val rotationThresholdTrigger: Trigger
        get() = Trigger { (xbox.rightX > 0.05) || (xbox.rightX < -0.05) }

    override val zeroEncoders: Button
        get() = Button { xbox.aButton }
    override val toggleBrakeMode: Button
        get() = Button { xbox.bButton }
    override val resetAll: Button
        get() = Button { xbox.xButton }
}
