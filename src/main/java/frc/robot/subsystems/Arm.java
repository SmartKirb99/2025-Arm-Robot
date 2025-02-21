// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final SparkMax m_armMotorOne = new SparkMax(ArmConstants.armMotorIds[0], MotorType.kBrushless);
  private final SparkMax m_armMotorTwo = new SparkMax(ArmConstants.armMotorIds[1], MotorType.kBrushless);
  private final RelativeEncoder m_encoderOne;
  private final RelativeEncoder m_encoderTwo;
  SparkMaxConfig m_armConfig = new SparkMaxConfig();
  SparkMaxConfig m_secondaryArmConfig = new SparkMaxConfig();


  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");
  private final GenericEntry m_angleDisplay;

  public boolean m_inMotion = false;
  private double m_setPoint = 0;

  /** Creates a new Arm. */
  public Arm() {
    m_encoderOne = m_armMotorOne.getEncoder();
    m_encoderTwo = m_armMotorTwo.getEncoder();
    m_encoderOne.setPosition(0);
    m_encoderTwo.setPosition(0);
    m_armConfig.inverted(true);
    m_secondaryArmConfig.inverted(false);
    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).getEntry();

    m_armMotorOne.configure(m_armConfig, null, null);
    m_armMotorTwo.configure(m_secondaryArmConfig, null, null);
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_armMotorOne.setVoltage(output);
    m_armMotorTwo.setVoltage(output);
  }

  /**
   * Gets the current angle of the arm.
   * @return Angle of the arm.
   */
  public double getAngle() {
    double[] positions = new double[2];
    positions[0] = m_encoderOne.getPosition();
    positions[1] = m_encoderTwo.getPosition(); 
    return positions;
  }

  /**
   * Sets the PID setpoint.
   * @param angle The desired angle of the arm.
   */
  public void setAngle(double angle) {
    m_setPoint = angle;
  }

  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  /**
   * Method for forcing the arm to move up.
   */
  public void up() {
    m_armMotor.set(-0.75); // Sets the speed of the motor to -3/4.
  }

  /**
   * Method for forcing the arm to move down.
   */
  public void down() {
    m_armMotor.set(0.75); // Sets the speed of the motor to 3/4.
  }

  public boolean getInMotion() {
    return m_inMotion;
  }

  public void setInMotion(boolean inMotion) {
    m_inMotion = inMotion;
  }


  public void setSpeed(double speed) {
    m_armMotor.set(speed);
  }

  public void stop(){
    m_armMotor.set(0);
  }

  @Override
  public void periodic() {
    m_angleDisplay.setDouble(getAngle());
    if(getAngle() < -5) { // Checks to see if the arm is past the min limit.
      setAngle(0); // If it is set the PID to 0.
    }
    if(getAngle() > ArmConstants.kMaxAngle) { // Checks to see if the arm is past the max limit.
      setAngle(ArmConstants.kMaxAngle); // If is is set the PID to 180.
    }
    if(getAngle() > m_setPoint - 0.1 && getAngle() < m_setPoint + 0.1) {
      m_inMotion = false;
    }
  }
}