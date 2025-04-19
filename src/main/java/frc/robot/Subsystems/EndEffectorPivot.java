// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class EndEffectorPivot extends SubsystemBase {

  private WarriorSparkMax pivotMotor;
  private DutyCycleEncoder pivotAbsEncoder;

  /** Creates a new EndEffectorPivot. */
  public EndEffectorPivot() {
    pivotMotor = 
      new WarriorSparkMax(
        Constants.EndEffectorConstants.PivotConstants.MotorConstants.END_EFFECTOR_PIVOT_MOTOR_ID, 
        MotorType.kBrushless, 
        Constants.EndEffectorConstants.PivotConstants.MotorConstants.END_EFFECTOR_PIVOT_MOTOR_INVERTED, 
        IdleMode.kCoast,
        Constants.EndEffectorConstants.PivotConstants.MotorConstants.END_EFFECTOR_PIVOT_MOTOR_CURRENT_LIMIT);

    pivotAbsEncoder =
      new DutyCycleEncoder(
        Constants.EndEffectorConstants.PivotConstants.EncoderConstants.PIVOT_ENCODER_DIO,
        Constants.EndEffectorConstants.PivotConstants.EncoderConstants.PIVOT_ENCODER_RANGE, 
        Constants.EndEffectorConstants.PivotConstants.EncoderConstants.PIVOT_ENCODER_OFFSET);

    pivotAbsEncoder.setInverted(Constants.EndEffectorConstants.PivotConstants.EncoderConstants.PIVOT_ENCODER_INVERTED);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder Position", this.getEncoderPosition());
    SmartDashboard.putNumber("Pivot Motor Voltage", this.getMotorVoltage());
    SmartDashboard.putNumber("Pivot Encoder Velocity", this.getVelocity());
  }

  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void setIdleMode(IdleMode idleMode) {
    pivotMotor.setIdleMode(idleMode);
  }

  public WarriorSparkMax getMotor() {
    return pivotMotor;
  }

  public DutyCycleEncoder getAbsEncoder() {
    return pivotAbsEncoder;
  }

  public double getEncoderPosition() {
    return pivotAbsEncoder.get();
  }

  public double getMotorVoltage() {
    return pivotMotor.getBusVoltage();
  }

  public double getVelocity() {
    return pivotMotor.getEncoder().getVelocity();
  }

}
