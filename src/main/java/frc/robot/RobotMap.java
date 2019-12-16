/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
  public static  WPI_TalonSRX FRTalonD = new WPI_TalonSRX(2); //Front right drive
  public static  WPI_TalonSRX FRTalonS = new WPI_TalonSRX(4); //front right steer
  public static  WPI_TalonSRX FLTalonD = new WPI_TalonSRX(6); //front left drive
  public static  VictorSPX FLTalonS = new VictorSPX(0); //front left steer
  public static  WPI_TalonSRX RRTalonD = new WPI_TalonSRX(7); //rear right drive
  public static  VictorSPX RRTalonS = new VictorSPX(1); //rear right steer
  public static  WPI_TalonSRX RLTalonD = new WPI_TalonSRX(3); //rear left drive
  public static  WPI_TalonSRX RLTalonS = new WPI_TalonSRX(1); //rear left steer

  public static Joystick leftJoy = new Joystick(0);
  public static Joystick rightJoy = new Joystick(1);

  public static DigitalInput frontRight = new DigitalInput(2);
  public static DigitalInput frontLeft = new DigitalInput(4);
  public static DigitalInput rearRight = new DigitalInput(0);
  public static DigitalInput rearLeft = new DigitalInput(6);

  public static Encoder swerveEncoder = new Encoder(0, 0);

  public static AHRS navx = new AHRS(SPI.Port.kMXP);
    }

