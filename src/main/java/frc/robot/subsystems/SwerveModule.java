// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final PIDController drivePIDController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController turnPIDController = new PIDController(0.0, 0.0, 0.0);

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
  private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
  
  public SwerveModule() {
    driveMotor = new CANSparkMax(0, MotorType.kBrushless);
    turnMotor = new CANSparkMax(0, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = driveMotor.getEncoder();

    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    driveEncoder.setPositionConversionFactor(0); 
    driveEncoder.setVelocityConversionFactor(0); // change to correct later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double currentRadians = getTurnPosition();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentRadians));

    double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
    double driveFeedForwardOutput = driveFeedForward.calculate(state.speedMetersPerSecond);
    double turnOutput = turnPIDController.calculate(currentRadians, state.angle.getRadians());
    double turnFeedForwardOutput = turnFeedForward.calculate(state.angle.getRadians());

    driveMotor.set(driveOutput + driveFeedForwardOutput);
    turnMotor.set(turnOutput + turnFeedForwardOutput);
  }

  public double getTurnPosition() {
    return turnEncoder.getPosition() * (Math.PI / 180);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(),
      new Rotation2d(getTurnPosition())
    );
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(), // Position of the wheel in meters or ticks, depending on how you set the conversion factor
      new Rotation2d(getTurnPosition()) // Angle of the wheel
    );
  }

  
}
