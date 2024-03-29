// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AmpConstants.kAmpIntakeSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

public class AmpIntake extends Command {
  /** Creates a new AmpIntake. */
  Amp m_amp;
  
  public AmpIntake(Amp amp) {
    m_amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_amp.setPercentOutput(kAmpIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_amp.setPercentOutput(kAmpIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_amp.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
