package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Talon leftTalon1 = new Talon(Constants.kLeftMotor1Port);
    private final Victor leftVictor1 = new Victor(Constants.kLeftMotor2Port);
    private final Victor leftVictor2 = new Victor(Constants.kLeftMotor3Port);

  private final Talon rightTalon1 = new Talon(Constants.kRightMotor1Port);
  private final Victor rightVictor1 = new Victor(Constants.kRightMotor2Port);
  private final Victor rightVictor2 = new Victor(Constants.kRightMotor3Port);

    public final DifferentialDrive m_drive = new DifferentialDrive(leftTalon1, rightTalon1);

    private final Encoder m_leftEncoder =
      new Encoder(Constants.kLeftEncoderPorts[0], Constants.kLeftEncoderPorts[1],
                  Constants.kLeftEncoderReversed);

    private final Encoder m_rightEncoder =
    new Encoder(Constants.kRightEncoderPorts[0], Constants.kRightEncoderPorts[1],
                Constants.kRightEncoderReversed);

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // private final Accelerometer accel = new 

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(gyro.getRotation2d(), m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftTalon1.setVoltage(leftVolts);
    rightTalon1.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void drive(double leftPower, double rightPower) {
    leftTalon1.set(leftPower);
    leftVictor1.set(leftPower);
    leftVictor2.set(leftPower);
    rightTalon1.set(rightPower);
    rightVictor1.set(rightPower);
    rightVictor2.set(rightPower);
}

public void stopDrive() {
    leftTalon1.set(0);
    leftVictor1.set(0);
    leftVictor2.set(0);
    rightTalon1.set(0);
    rightVictor1.set(0);
    rightVictor2.set(0);
}

public double getLeftEncoder() {
    return (m_leftEncoder.get() * Constants.DISTANCE_PER_PULSE);
}

public double getRightEncoder() {
    return (-m_rightEncoder.get() * Constants.DISTANCE_PER_PULSE);
}

public double getAverageEncoder() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
}

public void resetEncoder() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
}

public void driveWithMinPower(double leftPower, double rightPower, double minAbsolutePower) {
    double realLeftPower = (leftPower / Math.abs(leftPower)) * Math.max(Math.abs(leftPower), minAbsolutePower);
    double realRightPower = (rightPower / Math.abs(rightPower)) * Math.max(Math.abs(rightPower), minAbsolutePower);
}

}