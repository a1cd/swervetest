package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain


// new command base drive command class
class DriveCommand(
    private val drivetrain: Drivetrain,
    private val forward: Double = 0.0,
    private val strafe: Double = 0.0,
    private val rotation: Double = 0.0
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    // get from control scheme
    override fun execute() =
        drivetrain.move(
            forward,
            strafe,
            rotation
        )
    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) =
        drivetrain.move(0.0, 0.0, 0.0)
}