// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class collectBall extends CommandBase {
  private Intake collect;
  private Shooter shoot;
  private boolean frontSensorState;
  private boolean backSensorState;
  
  /** Creates a new collectBall. */

  public collectBall(Intake m_collect) {

    collect = m_collect;
    frontSensorState = false;
    backSensorState = false;

    
    // Use addRequirements() here to declare subsystem dependencies.
    //ddRequirements(collect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    collect.collect();
    
    /* frontSensorState = shoot.getSensorFrontState();
    / backSensorState = shoot.getSensorBackState();

    if(frontSensorState){
      
      shoot.ballIn();

      if(backSensorState){

        

      }
      


    }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    collect.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
