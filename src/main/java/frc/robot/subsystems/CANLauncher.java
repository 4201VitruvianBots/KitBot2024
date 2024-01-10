// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANLauncher extends SubsystemBase {
  TalonSRX m_launchWheel;
  TalonSRX m_feedWheel;

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_launchWheel = new TalonSRX(kLauncherID);
    m_feedWheel = new TalonSRX(kFeederID);
    
    TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
    SupplyCurrentLimitConfiguration motorLimitConfig = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
    
    motorConfig.slot0.kP = 0.24;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 0.0;
    
    // motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // //motorConfig.slot0.kV = 0.1185;

    // motorConfig.Voltage.PeakForwardVoltage = 12;
    // motorConfig.Voltage.PeakReverseVoltage = -12;

    // motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TO
    // // DO adjust this later
    // motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this later

    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    for (TalonSRX motor : new TalonSRX[] {m_launchWheel, m_feedWheel}) {
      motor.configFactoryDefault();
      
      motor.configSupplyCurrentLimit(motorLimitConfig);
      motor.configAllSettings(motorConfig);
    }
    
    // TODO: Apply configurations
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(TalonSRXControlMode.PercentOutput, speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(TalonSRXControlMode.PercentOutput, speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(TalonSRXControlMode.PercentOutput, 0);
    m_feedWheel.set(TalonSRXControlMode.PercentOutput, 0);
  }
}
