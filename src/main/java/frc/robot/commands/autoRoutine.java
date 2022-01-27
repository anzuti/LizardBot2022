// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoRoutine extends SequentialCommandGroup {

  private final DriveTrain m_train;
 
  /** Creates a new autoRoutine. */
  public autoRoutine(DriveTrain train) {

    m_train = train;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveDistancePID(m_train,5), 
      new rotateTo(m_train,90),
      new DriveDistancePID(m_train,4),
      new rotateTo(m_train, -45), 
      new DriveDistancePID(m_train, -5),
      new rotateTo(m_train, -45),
      new DriveDistancePID(m_train, -2.5));
  }
}
