// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

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
  private LaserCan laserSensor;
  private Measurement laserMeasurement;

  public Elevator() {

    elevatorMotor = 
        new WarriorSparkMax(
          Constants.ElevatorConstants.ELEVATOR_MOTOR_ID,
          MotorType.kBrushless,
          Constants.ElevatorConstants.ELEVATOR_MOTOR_INVERTED,
          IdleMode.kBrake,
          Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

    laserSensor =
        new LaserCan(Constants.ElevatorConstants.LASER_CAN_SENSOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    laserMeasurement = laserSensor.getMeasurement();
    SmartDashboard.putNumber("Elevator Height (Meters)", this.getHeightMeters());
  }

  public double getHeightMeters() {
    return laserMeasurement.distance_mm/1000;
  }

  public void setSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

}