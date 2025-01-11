package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// You got this
//01011001 01101111 01110101 00100000 01100111 01101111 01110100 00100000 01110100 01101000 01101001 01110011

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  /**
   * Runs when the robot is initialized, don't change
   */
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  /**
   * Runs periodically while the robot is on
   */
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  /**
   * Runs when the robot is first disabled, don't change as that means the robot when disabled runs code
   */
  public void disabledInit() {}

  @Override
  /**
   * Runs while the robot is disabled, no manual code or auton code here.
   */
  public void disabledPeriodic() {}

  @Override
  /**
   * First runs when the autonomous function is enabled
   */
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  @Override
  /**
   * Runs while autonomous is enabled, auton runs for 15 seconds anyways so no touch.
   */
  public void autonomousPeriodic() {}

  @Override
  /**
   * Runs when Teleop is first enabled
   */
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  @Override
  /**
   * Basically runs your code when TeleOP is enabled
   */
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  @Override
  public void testPeriodic() {}
}