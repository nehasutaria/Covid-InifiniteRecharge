/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.commands.IntakeSpinBack;
import frc.robot.subsystem.transport.Transport;
import frc.robot.subsystem.transport.commands.PushOut;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SpitBallsMode extends SequentialCommandGroup {
  /**
   * Creates a new SpitBalls.
   */
  public SpitBallsMode(Transport t, Intake i) {
    super(new PushOut(t), new IntakeSpinBack(i));
  }
}
