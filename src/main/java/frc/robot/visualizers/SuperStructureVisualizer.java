// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import static frc.robot.Constants.SimConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.*;

/** A class to visualize the state of all mechanisms on the robot. */
public class SuperStructureVisualizer {
  private Amp m_amp;
  private Climber m_climber;
  private Launcher m_launcher;

  Mechanism2d m_mech2d = new Mechanism2d(kDrivebaseWidth * 2, kDrivebaseWidth * 2);

  MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("Drivebase", kDrivebaseWidth * 0.5, kDrivebaseWidth * 0.5);
  MechanismRoot2d m_climberRoot2d =
      m_mech2d.getRoot(
          "Climber",
          kDrivebaseWidth * 0.5 + kClimberEdgeDistance,
          kDrivebaseWidth * 0.5);
  MechanismRoot2d m_launcherRoot2d =
      m_mech2d.getRoot(
          "Shooter",
          kDrivebaseWidth * 0.5 + kLauncherEdgeDistance,
          kDrivebaseWidth * 0.5);
  MechanismRoot2d m_ampRoot2d = 
        m_mech2d.getRoot(
            "Amp",
            kDrivebaseWidth * 0.5 + kAmpEdgeDistance,
            kDrivebaseWidth * 0.5);

  MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Drivebase", kDrivebaseWidth, 0));
  
  MechanismLigament2d m_climber2d =
      m_climberRoot2d.append(new MechanismLigament2d("Climber", kClimberUnextendedLength, 90));
  MechanismLigament2d m_climberHook1_2d =
      m_climber2d.append(new MechanismLigament2d("Hook 1", Units.inchesToMeters(3), -90));
  MechanismLigament2d m_climberHook2_2d =
      m_climberHook1_2d.append(new MechanismLigament2d("Hook 2", Units.inchesToMeters(3), -90));
  
  MechanismLigament2d m_launcherBase2d =
    m_launcherRoot2d.append(new MechanismLigament2d("Launcher Base", kLauncherBaseLength, kLauncherAngle));
  MechanismLigament2d m_launchWheel2d =
    m_launcherBase2d.append(new MechanismLigament2d("Launch Wheel", kLauncherWheelDiameter, 0));
  MechanismLigament2d m_launcherGap2d =
    m_launchWheel2d.append(new MechanismLigament2d("Launcher Gap", kLauncherWheelGapLength, 0));
  MechanismLigament2d m_feedWheel2d =
    m_launcherGap2d.append(new MechanismLigament2d("Feed Wheel", kLauncherWheelDiameter, 0));
  MechanismLigament2d m_launcherEnd2d =
    m_feedWheel2d.append(new MechanismLigament2d("Launcher End", kLauncherEndLength, 0));
  
  MechanismLigament2d m_ampBase2d =
    m_ampRoot2d.append(new MechanismLigament2d("Amp Base", kAmpBaseLength, 90));
  MechanismLigament2d m_ampWheel2d =
    m_ampBase2d.append(new MechanismLigament2d("Amp Wheel", kAmpWheelDiameter, 0));
  
  Color8Bit m_drivebase2d_originalColor,
      m_climber2d_originalColor,
      m_climberHook1_2d_originalColor,
      m_climberHook2_2d_originalColor,
      m_launcherBase2d_originalColor,
      m_launchWheel2d_originalColor,
      m_launcherGap2d_originalColor,
      m_feedWheel2d_originalColor,
      m_launcherEnd2d_originalColor,
      m_ampBase2d_originalColor,
      m_ampWheel2d_originalColor;
  
  public SuperStructureVisualizer(Amp amp, Climber climber, Launcher launcher) {
    registerAmp(amp);
    registerClimber(climber);
    registerLauncher(launcher);
    
    m_drivebase2d.setColor(new Color8Bit(0, 255, 0));
    m_climber2d.setColor(new Color8Bit(0, 255, 255));
    m_climberHook1_2d.setColor(new Color8Bit(0, 255, 255));
    m_climberHook2_2d.setColor(new Color8Bit(0, 255, 255));
    m_launcherBase2d.setColor(new Color8Bit(0, 255, 0));
    m_launchWheel2d.setColor(new Color8Bit(235, 137, 52));
    m_launcherGap2d.setColor(new Color8Bit(0, 255, 0));
    m_feedWheel2d.setColor(new Color8Bit(235, 137, 52));
    m_launcherEnd2d.setColor(new Color8Bit(0, 255, 0));
    m_ampBase2d.setColor(new Color8Bit(0, 255, 0));
    m_ampWheel2d.setColor(new Color8Bit(235, 137, 52));
    
    m_drivebase2d_originalColor = m_drivebase2d.getColor();
    m_climber2d_originalColor = m_climber2d.getColor();
    m_climberHook1_2d_originalColor = m_climberHook1_2d.getColor();
    m_climberHook2_2d_originalColor = m_climberHook2_2d.getColor();
    m_launcherBase2d_originalColor = m_launcherBase2d.getColor();
    m_launchWheel2d_originalColor = m_launchWheel2d.getColor();
    m_launcherGap2d_originalColor = m_launcherGap2d.getColor();
    m_feedWheel2d_originalColor = m_feedWheel2d.getColor();
    m_launcherEnd2d_originalColor = m_launcherEnd2d.getColor();
    m_ampBase2d_originalColor = m_ampBase2d.getColor();
    m_ampWheel2d_originalColor = m_ampWheel2d.getColor();

    SmartDashboard.putData("SuperStructure Sim", m_mech2d);
  }

  public void registerAmp(Amp amp) {
    m_amp = amp;
  }
  
  public void registerClimber(Climber climber) {
    m_climber = climber;
  }
  
  public void registerLauncher(Launcher launcher) {
    m_launcher = launcher;
  }

  /* Function to visualize the speed of a particular motor. */
  public void updateMotorColor(
      MechanismLigament2d ligament, double motorSpeed, Color8Bit originalColor) {
    double deltaBrightness = Math.abs(motorSpeed) * 75;

    Color8Bit newColor =
        new Color8Bit(
            originalColor.red + (int) deltaBrightness,
            originalColor.green + (int) deltaBrightness,
            originalColor.blue + (int) deltaBrightness);

    ligament.setColor(newColor);
  }

  public void updateAmp() {
    updateMotorColor(m_ampWheel2d, m_amp.getPercentOutput(), m_ampWheel2d_originalColor);
  }
  
  public void updateClimber() {
    updateMotorColor(m_climber2d, m_climber.getPercentOutput(), m_climber2d_originalColor);
    //m_climber2d.setLength(kClimberUnextendedLength + m_climber.getHeightMeters());
  }
  
  public void updateLauncher() {
    updateMotorColor(m_feedWheel2d, m_launcher.getFeedWheelPercentOutput(), m_feedWheel2d_originalColor);
    updateMotorColor(m_launchWheel2d, m_launcher.getLaunchWheelPercentOutput(), m_launchWheel2d_originalColor);
  }

  public void periodic() {
    if (m_amp != null) updateAmp();
    if (m_climber != null) updateClimber();
    if (m_launcher != null) updateLauncher();
  }
}
