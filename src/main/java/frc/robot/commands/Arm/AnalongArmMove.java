// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AnalongArmMove extends Command {
  private final Arm m_Arm;
  private DoubleSupplier m_upSpeed;
  private DoubleSupplier m_downSpeed;
  /** Creates a new AnalongArmMove. */
  public AnalongArmMove(DoubleSupplier downSpeed, DoubleSupplier upSpeed, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_upSpeed = upSpeed;
    m_downSpeed = downSpeed;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Nil I guess
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_upSpeed.getAsDouble() >= 0.1 || m_downSpeed.getAsDouble() >= 0.1){
      m_Arm.disable();
      m_Arm.setSpeed((m_upSpeed.getAsDouble() - m_downSpeed.getAsDouble()) * 0.6);
      m_Arm.setAngle(m_Arm.getAngle());
    } else {
      m_Arm.enable();
    }
    m_Arm.periodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.m_inMotion;
  }
}
