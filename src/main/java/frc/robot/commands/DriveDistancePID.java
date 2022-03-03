// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistancePID extends CommandBase {

  private double currentDistanceRightEncoder;
  private double currentDistanceLeftEncoder;
  private double averageEncoders;
  private final double distance;
  private final DriveTrain m_tren;
  private double startTime;

  /** Creates a new DriveDistancePID. */
  public DriveDistancePID(DriveTrain m_driveTrain, double d) {

    distance = d;
    m_tren = m_driveTrain;
    addRequirements(m_tren);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_tren.resetGyro();
    m_tren.resetLeftEncoder();
    m_tren.resetRightEncoder();
    m_tren.resetControllers();
    startTime = System.currentTimeMillis();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {


  currentDistanceLeftEncoder = m_tren.getLeftEncoder();
  currentDistanceRightEncoder = m_tren.getRightEncoder();  
  averageEncoders = (currentDistanceLeftEncoder + currentDistanceRightEncoder) / 2;  
    

    
    m_tren.driveToDistance(distance);



  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_tren.stopDriveTrain();
    m_tren.resetLeftEncoder();
    m_tren.resetRightEncoder();
    m_tren.resetControllers();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(averageEncoders - distance)<0.2;
  }
}
