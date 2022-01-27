// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class rotateTo extends CommandBase {
  
  private final DriveTrain m_train;
  private final double angle;
  private double currentDistanceRightEncoder;
  private double currentDistanceLeftEncoder;
  private double averageEncoders;


  /** Creates a new rotateTo. */
  public rotateTo(DriveTrain m_driveTrain, double a) {

    angle = a;
    m_train = m_driveTrain;
    addRequirements(m_train);
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_train.resetGyro();
    m_train.resetLeftEncoder();
    m_train.resetRightEncoder();
    m_train.resetControllers();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   /* currentDistanceLeftEncoder = Math.abs(m_train.getLeftEncoder());
    currentDistanceRightEncoder = Math.abs(m_train.getRightEncoder());  
    averageEncoders = (currentDistanceLeftEncoder + currentDistanceRightEncoder) / 2;  */
    m_train.rotate(angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_train.stopDriveTrain();
    m_train.resetControllers();
    m_train.resetGyro();
    m_train.resetLeftEncoder();
    m_train.resetRightEncoder();
    

  
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_train.getAngle()-angle)<3;
  }
}
