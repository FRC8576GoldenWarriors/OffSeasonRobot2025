// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private WarriorSparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;

  public Elevator() {

    elevatorMotor = 
        new WarriorSparkMax(
          Constants.ElevatorConstants.ELEVATOR_MOTOR_ID,
          MotorType.kBrushless,
          Constants.ElevatorConstants.ELEVATOR_MOTOR_INVERTED,
          IdleMode.kBrake,
          Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

    elevatorEncoder = elevatorMotor.getEncoder();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Position (Rotations)", getPosition());
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void setSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

}