// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private WarriorSparkMax pinchMotor;
  private DigitalInput coralDigitalInput;

  /** Creates a new EndEffector. */
  public EndEffector() {
    pinchMotor =
      new WarriorSparkMax(
        Constants.EndEffectorConstants.PinchConstants.PINCH_MOTOR_ID, 
        MotorType.kBrushless, 
        Constants.EndEffectorConstants.PinchConstants.PINCH_MOTOR_INVERTED, 
        IdleMode.kCoast,
        Constants.EndEffectorConstants.PinchConstants.PINCH_MOTOR_CURRENT_LIMIT);

    coralDigitalInput = new DigitalInput(Constants.EndEffectorConstants.PinchConstants.CORAL_DIGITAL_INPUT_DIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral Digital Input Has Coral", this.hasCoral());
    SmartDashboard.putBoolean("Pinch Motor Running", this.getMotorRunning());
    SmartDashboard.putNumber("Pinch Motor Velocity", this.getVelocity());
    SmartDashboard.putNumber("Pinch Motor Voltage", this.getMotorVoltage());
  }

  public boolean hasCoral() {
    return coralDigitalInput.get();
  }

  public double getVelocity() {
    return pinchMotor.getEncoder().getVelocity();
  }

  public double getMotorVoltage() {
    return pinchMotor.getBusVoltage();
  }

  public boolean getMotorRunning() {
    return pinchMotor.get() != 0;
  }

  public void setSpeed(double speed) {
    pinchMotor.set(speed);
  }

  public void setVoltage(double voltage) {
    pinchMotor.setVoltage(voltage);
  }
}
