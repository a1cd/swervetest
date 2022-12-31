package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain


/**
 * The default drive command. Arcade drive based on xbox controller inputs
 */
class DriveCommand(
    private val drivetrain: Drivetrain,
    private val forward: Double,
    private val strafe: Double,
    private val rotation: Double
): CommandBase() {
    constructor(drivetrain: Drivetrain, controlScheme: ControlScheme): this(
        drivetrain,
        controlScheme.forward,
        controlScheme.strafe,
        controlScheme.rotation
    )
    init {
        addRequirements(drivetrain)
    }
    // get from control scheme
    override fun execute() {
        drivetrain.move(
            forward,
            strafe,
            rotation
        )
        println("forward: $forward, strafe: $strafe, rotation: $rotation")
    }
    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        drivetrain.move(0.0, 0.0, 0.0)
    }
}