// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.utils.CtreUtils.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase implements AutoCloseable {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;

  TalonFX leftFront;
  TalonFX leftRear;
  TalonFX rightFront;
  TalonFX rightRear;

  Pigeon2 pigeon = new Pigeon2(kPigeonID);
  DifferentialDriveOdometry odometry;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {
    pigeon.setYaw(0);

    leftFront = new TalonFX(kLeftFrontID);
    leftRear = new TalonFX(kLeftRearID);
    rightFront = new TalonFX(kRightFrontID);
    rightRear = new TalonFX(kRightRearID);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    TalonFXConfiguration motorConfig = generateDriveMotorConfig();
    leftFront.getConfigurator().apply(motorConfig);
    leftRear.getConfigurator().apply(motorConfig);
    rightFront.getConfigurator().apply(motorConfig);
    rightRear.getConfigurator().apply(motorConfig);

    // Set the rear motors to follow the front motors.
    leftRear.setControl(new Follower(leftFront.getDeviceID(), false));
    rightRear.setControl(new Follower(rightFront.getDeviceID(), false));

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    odometry =
        new DifferentialDriveOdometry(
            pigeon.getRotation2d(), getDistanceMeters(true), getDistanceMeters(false));
  }

  private double getDistanceMeters(boolean left) {
    if (left) {
      return leftFront.getRotorPosition().getValueAsDouble()
          * kGearRatio
          * Math.PI
          * kWheelDiameter;
    } else {
      return rightFront.getRotorPosition().getValueAsDouble()
          * kGearRatio
          * Math.PI
          * kWheelDiameter;
    }
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void resetGyro() {
    pigeon.setYaw(0);
    // TODO: Figure out how to set accum z angle
    // TODO: Reset odometry
  }

  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
    odometry.update(pigeon.getRotation2d(), getDistanceMeters(true), getDistanceMeters(false));
  }

  @Override
  public void close() {
    leftRear.close();
    rightRear.close();
  }
}
