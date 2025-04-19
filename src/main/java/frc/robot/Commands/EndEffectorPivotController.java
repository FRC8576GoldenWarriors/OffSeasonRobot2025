// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.EndEffectorPivot;

public class EndEffectorPivotController extends Command {

  EndEffectorPivot endEffectorPivot;

  double setpoint;
  double voltage;

  PIDController pivotPID;
  ArmFeedforward pivotFF;

  double PIDVoltage;
  double FFVoltage;

  double COMOffset;

  public EndEffectorPivotController(EndEffectorPivot endEffectorPivot, double setpoint) {

    this.endEffectorPivot = endEffectorPivot;
    this.setpoint = setpoint;

    pivotPID = 
        new PIDController(
          Constants.EndEffectorConstants.PivotConstants.kP,
          Constants.EndEffectorConstants.PivotConstants.kI, 
          Constants.EndEffectorConstants.PivotConstants.kD);

    pivotFF =
        new ArmFeedforward(
          Constants.EndEffectorConstants.PivotConstants.kS, 
          Constants.EndEffectorConstants.PivotConstants.kG, 
          Constants.EndEffectorConstants.PivotConstants.kV,
          Constants.EndEffectorConstants.PivotConstants.kA);

    COMOffset = Constants.EndEffectorConstants.PivotConstants.COMOffset;

    addRequirements(endEffectorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //FF angle radians //IDK HOW TO DO ROTATIONAL FF
    FFVoltage = pivotFF.calculate(((setpoint + COMOffset - 0.25) * Math.PI * 2), 2.0);

    PIDVoltage = pivotPID.calculate(endEffectorPivot.getEncoderPosition(), setpoint);

    voltage = FFVoltage + PIDVoltage;

    //soft stops
    if (endEffectorPivot.getEncoderPosition() > 0.75 || endEffectorPivot.getEncoderPosition() < 0.2){
      voltage = 0;
    }

    endEffectorPivot.setVoltage(voltage);

    SmartDashboard.putNumber("Pivot FF Voltage", FFVoltage);
    SmartDashboard.putNumber("Pivot PID Voltage", PIDVoltage);
    SmartDashboard.putNumber("Pivot Voltage", voltage);
    SmartDashboard.putBoolean("Pivot At Setpoint", pivotPID.atSetpoint());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffectorPivot.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
