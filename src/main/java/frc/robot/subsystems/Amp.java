// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AmpConstants.*;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */
  TalonSRX m_ampMotor;
  DigitalInput distanceSensorDigitalInput = new DigitalInput(1);
  DigitalInput distanceSensorDigitalInput2 = new DigitalInput(2);
  public Amp() {
    m_ampMotor = new TalonSRX(kAmpID);
    
    m_ampMotor.configFactoryDefault();
    m_ampMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    //m_ampMotor.configPeakCurrentLimit(kAmpCurrentLimit);
  }

  public void setSpeed(double output) {
    m_ampMotor.set(TalonSRXControlMode.PercentOutput, output);
  }
  public boolean getSensorInput1() {
    return distanceSensorDigitalInput.get();
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
