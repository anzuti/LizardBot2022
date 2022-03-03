// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;

public class release extends CommandBase {
  private Lift myLift;


  /** Creates a new shoot. */

  public release(Lift m_lift) {

    myLift = m_lift;
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(Shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    myLift.releaseWinch();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Shoot.stopIndexer();
    myLift.stopWinch();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
