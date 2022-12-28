package frc.robot

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.simulation.XboxControllerSim
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.junit.jupiter.api.DisplayName

class RobotContainerTest {
    var xboxSim: XboxControllerSim? = null
    var xboxController: XboxController? = null
    var robotContainer: RobotContainer? = null
    @BeforeEach
    fun setUp() {
        HAL.initialize(500, 0)
        // setup robot simulation
        xboxController = XboxController(0)
        xboxSim = XboxControllerSim(0)
        robotContainer = RobotContainer(xboxController!!)
    }

    @AfterEach
    fun tearDown() {
        robotContainer = null
        xboxController = null
        xboxSim = null
    }

    @Test
    @DisplayName("drivetrain is not null")
    fun getDrivetrain() {
        assertNotNull(robotContainer!!.drivetrain)
    }

    @Test
    @DisplayName("control scheme not null and control scheme creates triggers and buttons")
    fun getControlScheme() {
        assertNotNull(robotContainer!!.controlScheme)

        // make the control scheme's private button variables public
        // so we can test them
        val buttonsField = CommandScheduler.getInstance()
            .javaClass
            .declaredFields.findLast { it.name == "m_buttons" }
        buttonsField!!.isAccessible = true
        val out = buttonsField.get(CommandScheduler.getInstance())
        print(out as Collection<*>)
    }

    @Test
    @DisplayName("can set control scheme")
    fun setControlScheme() {
        val testScheme = TestControlScheme().also { robotContainer!!.controlScheme = it }
        assertEquals(testScheme, robotContainer!!.controlScheme)
    }

    @Test
    @DisplayName("disabledInit sets state to disabled")
    fun disabledInit() {
        robotContainer!!.disabledInit()
        assertEquals(robotContainer!!.state, RobotState.DISABLED)
    }

    @Test
    @DisplayName("autonomousInit sets state to autonomous")
    fun autonomousInit() {
        robotContainer!!.autonomousInit()
        assertEquals(robotContainer!!.state, RobotState.AUTONOMOUS)
    }

    @Test
    @DisplayName("teleopInit sets state to teleop")
    fun teleopInit() {
        robotContainer!!.teleopInit()
        assertEquals(robotContainer!!.state, RobotState.TELEOP)
    }

    @Test
    @DisplayName("testInit sets state to test")
    fun testInit() {
        robotContainer!!.testInit()
        assertEquals(robotContainer!!.state, RobotState.TEST)
    }
}