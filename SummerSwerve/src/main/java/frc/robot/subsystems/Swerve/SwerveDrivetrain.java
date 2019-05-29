/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class SwerveDrivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_TalonSRX frDrive = RobotMap.FRTalonD; //Front right drive
  public static WPI_TalonSRX frSteer = RobotMap.FRTalonS; //front right steer
  public static WPI_TalonSRX flDrive = RobotMap.FLTalonD; //front left drive
  public static WPI_TalonSRX flSteer = RobotMap.FLTalonS; //front left steer
  public static WPI_TalonSRX rrDrive = RobotMap.RRTalonD; //rear right Drive
  public static WPI_TalonSRX rrSteer = RobotMap.RRTalonS; //rear right steer
  public static WPI_TalonSRX rlDrive = RobotMap.RLTalonD; //rear left drive
  public static WPI_TalonSRX rlSteer = RobotMap.RLTalonS; //rear left steer

  private final double width = 1;
  private final double length = 1;

  public Swerve swerveDrivetrain;

  private SwerveModule frontRight;
  private SwerveModule frontLeft;
  private SwerveModule rearRight;
  private SwerveModule rearLeft;

  private AnalogInput analogFrontRight = RobotMap.frontRight;
  private AnalogInput analogFrontLeft = RobotMap.frontLeft;
  private AnalogInput analogRearRight = RobotMap.rearRight;
  private AnalogInput analogRearLeft = RobotMap.rearLeft;

  private static Encoder enc = RobotMap.swerveEncoder;
  private static AHRS gyro = RobotMap.navx;

  private final double P = 1;
  private final double I = 0;
  private final double D = 0;

public SwerveDrivetrain(){
  frontLeft = new SwerveModule("Front Left Wheel", flDrive, flSteer);
  frontRight = new SwerveModule("Front Right Wheel", frDrive, flSteer);
  rearLeft = new SwerveModule("Rear Left Wheel", rlDrive, rlSteer);
  rearRight = new SwerveModule("Rear Right Wheel", rrDrive, rrSteer);

  init();

  swerveDrivetrain = new Swerve(frontRight, frontLeft, rearRight, rearLeft, width, length);
}

  @Override
  public void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive());
  }

  public void init(){
   //set gyro to 0;
   
   initSteerMotor(frSteer);
   initSteerMotor(flSteer);
   initSteerMotor(rrSteer);
   initSteerMotor(rlSteer);

   resetQuadrentureEncoder();
   resetDriveEnc();
  }

  public void initSteerMotor(WPI_TalonSRX steerMotor){
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    steerMotor.configPeakOutputForward(1, 10);
    steerMotor.configPeakOutputReverse(-1, 10);

    steerMotor.configNominalOutputForward(0, 10);
    steerMotor.configNominalOutputReverse(0, 10);

    steerMotor.setNeutralMode(NeutralMode.Brake);

    steerMotor.configAllowableClosedloopError(0, 4, 10);

    steerMotor.config_kP(0, P);
    steerMotor.config_kI(0, I);
    steerMotor.config_kD(0, D);
  }

  public void resetDriveEnc(){
    enc.reset();
  }

  public void resetQuadrentureEncoder(){
    //frSteer.setSelectedSensorPosition(analogFrontRight.getValue()) setting sensor positions to a constant value for starting (doesn't seem to be 0)
    
    frSteer.set(ControlMode.Position, 0);
    flSteer.set(ControlMode.Position, 0);
    rrSteer.set(ControlMode.Position, 0);
    rlSteer.set(ControlMode.Position, 0);
  }

public double getDistance(){
  return enc.getDistance();
}

public double getGyro(){
  return gyro.getAngle();
}

public void move(double fwd, double str, double rcw){
  swerveDrivetrain.move(fwd, str, rcw, getGyro());
}

public void stop(){
  swerveDrivetrain.stop();
}

}
