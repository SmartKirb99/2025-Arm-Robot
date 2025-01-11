// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpArm extends Command {
  private  final Arm m_Arm;
  private boolean m_pastLimit = false;
  /** Creates a new UpArm. */
  public UpArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Arm = arm;
  addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_pastLimit){
      m_Arm.setAngle(0);
    } else {
      m_Arm.setAngle(m_Arm.getAngle());
    }
    m_Arm.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Arm.getAngle() < 0){
      m_pastLimit = true;
      return true;
    } else {
      return false;
    }
  }
}
