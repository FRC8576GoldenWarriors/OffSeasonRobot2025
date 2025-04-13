// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;

public class ElevatorController extends Command {
  private Elevator elevator;
  private double setpoint;
  private double speed;

  private PIDController elevatorPID;
  private ElevatorFeedforward elevatorFF;

  private double PIDvoltage;
  private double FFvoltage;
  private double voltage;

  public ElevatorController(Elevator elevator, double setpoint, double speed) {

    this.elevator = elevator;

    this.setpoint = setpoint;
    this.speed = speed;

    elevatorPID = 
        new PIDController(
          Constants.ElevatorConstants.kP,
          Constants.ElevatorConstants.kI, 
          Constants.ElevatorConstants.kP);
    elevatorPID.setTolerance(Constants.ElevatorConstants.ALLOWED_DISTANCE_ERROR);

    elevatorFF =
        new ElevatorFeedforward(
          Constants.ElevatorConstants.kS, 
          Constants.ElevatorConstants.kG, 
          Constants.ElevatorConstants.kV);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Idk how to do feedforward
    FFvoltage = elevatorFF.calculate(speed);
    PIDvoltage = elevatorPID.calculate(elevator.getHeightMeters(), setpoint);
    
    voltage = PIDvoltage + FFvoltage;

    elevator.setVoltage(voltage);
    elevator.setSpeed(speed);

    SmartDashboard.putNumber("Elevator voltage ouput", voltage);
    SmartDashboard.putNumber("Elevator PID voltage output", PIDvoltage);
    SmartDashboard.putNumber("Elevator FF voltage output", FFvoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
    elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
