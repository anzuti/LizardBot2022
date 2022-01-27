// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ejectBall extends CommandBase {
  private Intake collect;
  
  /** Creates a new collectBall. */

  public ejectBall(Intake m_collect) {

    collect = m_collect;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //collect.stopIntake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    collect.eject();

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
