// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ClimberConstants.kClimberSpeed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimberJoystick extends Command {
  Climber m_climber;
  DoubleSupplier m_joystickY;
  
  /** Creates a new RunClimberJoystick. */
  public RunClimberJoystick(Climber climber, DoubleSupplier joystickY) {
    m_climber = climber;
    m_joystickY = joystickY;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setPercentOutput(-(kClimberSpeed * MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.05)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
