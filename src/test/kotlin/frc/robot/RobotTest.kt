package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.simulation.XboxControllerSim
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.RunCommand
import org.hamcrest.CoreMatchers.nullValue
import org.junit.After
import org.junit.Assert.*
import org.junit.Assume.assumeThat
import org.junit.Before
import org.junit.Test
import org.junit.Assume.assumeFalse as jAssumeFalse
import org.junit.Assume.assumeTrue as jAssumeTrue

private fun assumeFalse(condition: Boolean, message: String) = jAssumeFalse(message, condition)
private fun assumeTrue(condition: Boolean, message: String) = jAssumeTrue(message, condition)
/**
 * Test the robot class.
 */
class RobotTest {
    // robot
    var robot: Robot = Robot()

    // xboxSim controller
    private var xboxSim: XboxControllerSim? = null
    private var xboxController: XboxController? = null


    // simulation
    class TestRobotContainer(xbox: XboxController) : RobotContainer(xbox) {
        var lastCommand: String? = null
        override fun periodic() {
            lastCommand = "periodic"
        }
        override fun disabledInit() {
            lastCommand = "disabledInit"
        }
        override fun disabledPeriodic() {
            lastCommand = "disabledPeriodic"
        }
        override fun autonomousInit() {
            lastCommand = "autonomousInit"
        }
        override fun autonomousPeriodic() {
            lastCommand = "autonomousPeriodic"
        }
        override fun teleopInit() {
            lastCommand = "teleopInit"
        }
        override fun teleopPeriodic() {
            lastCommand = "teleopPeriodic"
        }
        override fun testInit() {
            lastCommand = "testInit"
        }
        override fun testPeriodic() {
            lastCommand = "testPeriodic"
        }
        override fun simulationPeriodic() {
            lastCommand = "simulationPeriodic"
        }
        override fun robotInit() {
            lastCommand = "robotInit"
        }
    }
    private var testRobotContainer: TestRobotContainer? = null
    @Before
    fun setUp() {
        // setup robot simulation
        robot = Robot()
        // setup xboxSim controller simulation
        xboxSim = XboxControllerSim(0)
        xboxController = XboxController(0)
        // setup robot container
        testRobotContainer = TestRobotContainer(xboxController!!)
    }
    @After
    fun tearDown() {
        // stop robot simulation
        this.robot.disabledInit()
        // stop xboxSim controller simulation
        this.xboxSim = null
        this.xboxController = null
        // stop robot container
        this.testRobotContainer = null
    }
    @Test
//    @DisplayName("test that robot container is initialized")
    fun robotInit() {
        assumeThat(
            "robot container should be null",
            this.robot.robotContainer,
            nullValue()
        )
        // test robot init
        this.robot.robotInit()
        assertNotNull(this.robot.robotContainer)
    }
    @Test
    @org.junit.Ignore("bad test") // TODO: Remove this test
//    @DisplayName("test robot periodic - check that robot advances command scheduler")
    fun robotPeriodic() {
        // -- before --
        // test command scheduler using a command and time since it was scheduled
        class TestCommand: RunCommand({}) {
            var hasExecuted = false
            override fun execute() {
                super.execute()
                this.hasExecuted = true
            }
            override fun isFinished(): Boolean = false
        }
        CommandScheduler.getInstance().cancelAll()
        val testCommand = TestCommand().apply {
            CommandScheduler.getInstance().schedule(this)
        }
        assumeTrue(
            CommandScheduler.getInstance().timeSinceScheduled(testCommand) <= 0.0,
            "test command should not have executed and should be zero time since scheduled"
        )
        // test robot periodic
        for (i in 0..10) {
            this.robot.robotPeriodic()
            Thread.sleep(100)
        }
        // -- after --
        assertTrue(CommandScheduler.getInstance().timeSinceScheduled(testCommand) > 0.0001)
        assertTrue(testCommand.hasExecuted)
    }
    @Test
    @org.junit.Ignore("bad test") // TODO: Remove this test
//    @DisplayName("test motor power is zero when disabled")
    fun disabledInit() {
        // advance simulation time
        robot.robotInit()
        robot.robotPeriodic()

        // switch sim robot to test mode
        robot.testInit()
        robot.testPeriodic()

        // make sure motor
        assumeFalse(
            robot.robotContainer == null,
            "robot container is null"
        )
        // motor speed is zero
        assumeTrue(
            0.0== robot.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0,
            "motor speed should be 0.0 before test"
        )

        // move the robot using the test control scheme
        xboxSim!!.setRawAxis(0, 0.5)
        xboxSim!!.setRawAxis(1, 0.5)

        // advance the simulation
        for (i in 0..5) {
            robot.robotPeriodic()
            Thread.sleep(100)
        }

        // check motor percent outputs are not zero with some tolerance
        assumeFalse(
            robot.robotContainer == null,
            "robot container is null"
        )
        assumeFalse(
            0.01 > Math.abs(robot.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0),
            "motor speed should increase after moving xbox joystick"
        )

        // finally actually test to see if the robot properly disables
        // test robot disabled init
        robot.disabledInit()

        // advance the simulation
        for (i in 0..15) {
            robot.robotPeriodic()
            robot.disabledPeriodic()
            Thread.sleep(100)
        }
        assumeFalse(
            robot.robotContainer==null,
            "robot container is null"
        )
        assertEquals(
            "motor power should be zero after disabling",
            0.0,
            robot.robotContainer!!.drivetrain.fl.driveMotor.selectedSensorVelocity * 10.0 / 4096.0,
            0.01
        )
    }
    @Test
//    @DisplayName("Test robot container disabled init is called when robot is disabled")
    fun disabledInitSubcallTest() {
        this.robot.robotContainer = testRobotContainer
        this.robot.disabledInit()
        assertEquals(
            "disabledInit() should call robotContainer.disabledInit()",
            "disabledInit",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robot container disabled periodic is called when robot is disabled")
    fun disabledPeriodic() {
        this.robot.robotContainer = testRobotContainer
        this.robot.disabledPeriodic()
        assertEquals(
            "disabledPeriodic() should call robotContainer.disabledPeriodic()",
            "disabledPeriodic",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.autonomousInit() called when autonomousInit() is called")
    fun testInit() {
        this.robot.robotContainer = testRobotContainer
        this.robot.testInit()
        assertEquals(
            "testInit() should call robotContainer.testInit()",
            "testInit",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.testPeriodic() called when testPeriodic() is called")
    fun testPeriodic() {
        this.robot.robotContainer = testRobotContainer
        this.robot.testPeriodic()
        assertEquals(
            "testPeriodic() should call robotContainer.testPeriodic()",
            "testPeriodic",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.autonomousInit() called when autonomousInit() is called")
    fun autonomousInit() {
        this.robot.robotContainer = testRobotContainer
        this.robot.autonomousInit()
        assertEquals(
            "autonomousInit() should call robotContainer.autonomousInit()",
            "autonomousInit",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.autonomousPeriodic() called when autonomousPeriodic() is called")
    fun autonomousPeriodic() {
        this.robot.robotContainer = testRobotContainer
        this.robot.autonomousPeriodic()
        assertEquals(
            "autonomousPeriodic() should call robotContainer.autonomousPeriodic()",
            "autonomousPeriodic",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.teleopInit() called when teleopInit() is called")
    fun teleopInit() {
        this.robot.robotContainer = testRobotContainer
        this.robot.teleopInit()
        assertEquals(
            "teleopInit() should call robotContainer.teleopInit()",
            "teleopInit",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.teleopPeriodic() called when teleopPeriodic() is called")
    fun teleopPeriodic() {
        this.robot.robotContainer = testRobotContainer
        this.robot.teleopPeriodic()
        assertEquals(
            "teleopPeriodic() should call robotContainer.teleopPeriodic()",
            "teleopPeriodic",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.robotPeriodic() called when robotPeriodic() is called")
    fun simulationPeriodic() {
        this.robot.robotContainer = testRobotContainer
        this.robot.simulationPeriodic()
        assertEquals(
            "simulationPeriodic() should call robotContainer.simulationPeriodic()",
            "simulationPeriodic",
            testRobotContainer!!.lastCommand,
        )
    }
    @Test
//    @DisplayName("Test robotContainer.robotPeriodic() called when robotPeriodic() is called")
    fun simulationInit() {
        this.robot.robotContainer = testRobotContainer
        this.robot.simulationInit()
        assertEquals(
            "simulationInit() should have set robotContainer to the normal robotContainer.",
            RobotContainer(xboxController!!).javaClass,
            robot.robotContainer?.javaClass
        )
    }
}
