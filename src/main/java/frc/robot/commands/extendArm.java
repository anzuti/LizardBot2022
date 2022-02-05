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
  private Lift Lift;
  

  /** Creates a new extendArm */

  public extendArm(Lift m_lift) {

    Lift = m_lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   Lift.extendArm(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Lift.extendArm(Constants.dampenSpeedLift);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

   Lift.extendArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Lift.getLimitSwitch()){

      return true;
    }
    else
    return false;
  }
}
