// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class extendArm extends CommandBase {
  private Lift myLift;
  private double speed;
  

  /** Creates a new extendArm */

  public extendArm(Lift m_lift, double m_speed) {

    myLift = m_lift;
    speed = m_speed;
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   myLift.extendArm(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    myLift.extendArm(speed);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

   myLift.extendArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   /* if(myLift.getLimitSwitch()){

      return true;
    }
    else
  
  }*/
  return false;
}

}