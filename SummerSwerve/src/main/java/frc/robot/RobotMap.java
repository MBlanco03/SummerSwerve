/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static  WPI_TalonSRX FRTalonD = new WPI_TalonSRX(0); //Front right drive
  public static  WPI_TalonSRX FRTalonS = new WPI_TalonSRX(0); //front right steer
  public static  WPI_TalonSRX FLTalonD = new WPI_TalonSRX(0); //front left drive
  public static  WPI_TalonSRX FLTalonS = new WPI_TalonSRX(0); //front left steer
  public static  WPI_TalonSRX RRTalonD = new WPI_TalonSRX(0); //rear right drive
  public static  WPI_TalonSRX RRTalonS = new WPI_TalonSRX(0); //rear right steer
  public static  WPI_TalonSRX RLTalonD = new WPI_TalonSRX(0); //rear left drive
  public static  WPI_TalonSRX RLTalonS = new WPI_TalonSRX(0); //rear left steer

  public static Joystick leftJoy = new Joystick(0);
  public static Joystick rightJoy = new Joystick(1);
  public static DigitalInput[] encoder = new DigitalInput[8];
  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  public RobotMap(){
    for(int e = 0; e < 8; e++){
      encoder[e] = new DigitalInput(e);
    }
  }
    }

