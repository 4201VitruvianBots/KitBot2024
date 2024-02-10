// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  TalonSRX m_launchWheel;
  TalonSRX m_feedWheel;
  DigitalInput distanceSensorDigitalInput = new DigitalInput(1);
  DigitalInput distanceSensorDigitalInput2 = new DigitalInput(2);

  /** Creates a new Launcher. */
  public Launcher() {
    m_launchWheel = new TalonSRX(kLauncherID);
    m_feedWheel = new TalonSRX(kFeederID);

    m_launchWheel.configFactoryDefault();
    m_feedWheel.configFactoryDefault();

    m_launchWheel.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    m_feedWheel.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    // m_launchWheel.configPeakCurrentLimit(kLauncherCurrentLimit);
    // m_feedWheel.configPeakCurrentLimit(kFeedCurrentLimit);
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (distanceSensorDigitalInput.get()) {
      System.out.println("Sensor 1 Working");
    }
    else {
      System.out.println("Sensor 1 Not Working");
    }
    if (distanceSensorDigitalInput2.get()) {
      System.out.println("Sensor 2 Working");
    }
    else {
      System.out.println("Sensor 2 Not Working");
    }
  }

}
