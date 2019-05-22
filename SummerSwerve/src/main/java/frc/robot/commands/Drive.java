/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Drive extends Command {

  private static double fwd;
  private static double str;
  private static double rcw;

  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    fwd = Robot.oi.getLeftJoy().getY();
    str = Robot.oi.getLeftJoy().getX();
    rcw = Robot.oi.getRightJoy().getY();

    if (fwd < 0){
      fwd *= fwd * -1;
    }
    else {
      fwd *= fwd;
    }
    if(str < 0){
      str *= str*-1;
      }
    else{
      str *= str;
    }
    if(rcw < 0){
      rcw *= rcw * -1;
    }
    else{
      rcw  *= rcw;
    }

    Robot.drivetrain.move(fwd, str, rcw);

    }
    
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
