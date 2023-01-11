package frc.robot.util

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.sim.PhysicsSim
import org.junit.Assume

open class SimulatedRobot(
    private var test: SimulatedRobotTest<out SimulatedRobot>
): TimedRobot() {
    override fun robotInit() {
        super.robotInit()
        Assume.assumeNotNull(test.robotInitLatch)
        this.test.robotInitLatch?.countDown()
    }
    override fun robotPeriodic() {
//        CommandScheduler.getInstance().run()
        // is command scheduler code thread safe?
        // A: no, because it is not synchronized
        // can you fix it
        // A: yes:
        // new lock
         synchronized(CommandScheduler.getInstance()) {
            CommandScheduler.getInstance().run()
         }
    }
    override fun simulationPeriodic() {
        PhysicsSim.instance.run()
    }
}