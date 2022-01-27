// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class simpleDriveDistance extends CommandBase {

  private final double distance;
  private double currentDistanceLeftEncoder;
  private double currentDistanceRightEncoder;
  private double averageEncoders;
  private final DriveTrain m_tren;

  

  /** Creates a new simpleDriveDistance. */
  public simpleDriveDistance(DriveTrain m_driveTrain, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
  distance = d;
  m_tren = m_driveTrain;
  addRequirements(m_tren);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    m_tren.resetLeftEncoder();
    m_tren.resetRightEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    currentDistanceLeftEncoder = m_tren.getLeftEncoder();
    currentDistanceRightEncoder = m_tren.getRightEncoder();

    averageEncoders = (currentDistanceRightEncoder + currentDistanceLeftEncoder)/2;

    

    if(averageEncoders<distance)
    {

      m_tren.driveStraight(0.7);

    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return averageEncoders >= distance;
  }
}
