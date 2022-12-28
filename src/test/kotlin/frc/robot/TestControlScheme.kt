package frc.robot

import edu.wpi.first.wpilibj2.command.button.Button
import frc.robot.controls.ControlScheme

class TestControlScheme: ControlScheme {
    var zeroEncodersValue = false
    var toggleBrakeModeValue = false
    var resetAllValue = false

    override val zeroEncoders = Button {zeroEncodersValue}
    override val toggleBrakeMode = Button {toggleBrakeModeValue}
    override val resetAll = Button {resetAllValue}

    override val forward = 0.0
    override val strafe = 0.0
    override val rotation = 0.0
}