// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  TalonFX m_climberMotor = new TalonFX(kClimberID);
  
  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  public void setPercentOutput(double output) {
    m_climberMotor.set(output);
  }
  
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Speed", m_climberMotor.get());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
