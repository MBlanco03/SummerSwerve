/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import java.lang.Math;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwerveMath {
    public static  WPI_TalonSRX frDrive = RobotMap.FRTalonD; //Front right drive
    public static  WPI_TalonSRX frSteer = RobotMap.FRTalonS; //front right steer
    public static  WPI_TalonSRX flDrive = RobotMap.FLTalonD; //front left drive
    public static  WPI_TalonSRX flSteer = RobotMap.FLTalonS; //front left steer
    public static  WPI_TalonSRX rrDrive = RobotMap.RRTalonD; //rear right Drive
    public static WPI_TalonSRX rrSteer = RobotMap.RRTalonS; //rear right steer
    public static  WPI_TalonSRX rlDrive = RobotMap.RLTalonD; //rear left drive
    public static  WPI_TalonSRX rlSteer = RobotMap.RLTalonS; //rear left steer
   
    private static Joystick leftJoy = OI.getLeftJoy();
    private static Joystick rightJoy = OI.getRightJoy();
    private static AHRS gyro = RobotMap.navx;

    public SwerveMath(){
        frSteer.set(ControlMode.Position, 0);
        flSteer.set(ControlMode.Position, 0);
        rrSteer.set(ControlMode.Position, 0);
        rlSteer.set(ControlMode.Position, 0);
    
        frDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        flDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        rrDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        rlDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        frSteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        flSteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        rrSteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        rlSteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
       
        frSteer.config_kP(0, 1);
        frSteer.config_kI(0, 0);
        frSteer.config_kD(0, 0);

        flSteer.config_kP(0, 1);
        flSteer.config_kI(0, 0);
        flSteer.config_kD(0, 0);

        rrSteer.config_kP(0, 1);
        rrSteer.config_kI(0, 0);
        rrSteer.config_kD(0, 0);

        rlSteer.config_kP(0, 1);
        rlSteer.config_kI(0, 0);
        rlSteer.config_kD(0, 0);
    }

    public void move(){
    double fwd = leftJoy.getY();
    double str = leftJoy.getX();
    double rcw = rightJoy.getX();
    double length = 1.0;
    double width = 1.0;
    double diagonal = Math.sqrt(Math.pow(length, 2) + Math.pow(width, 2));

    double A;
    double B;
    double C;
    double D;

    double frs;
    double fls;
    double rrs;
    double rls;

    double fra;
    double fla;
    double rra;
    double rla;
    double fwd2;
    double str2;
    double temp;
    double max;

    temp = (fwd*Math.cos(gyro.getAngle())) + str*Math.sin(gyro.getAngle());
    str2 = (-fwd*Math.sin(gyro.getAngle())) + str*Math.cos(gyro.getAngle());
    fwd2 = temp;

    A = str2 - rcw*(length/diagonal);
    B = str2 + rcw*(length/diagonal);
    C = fwd2 - rcw*(width/diagonal);
    D = fwd2 + rcw*(width/diagonal);

    frs = Math.sqrt((Math.pow(B,2)+Math.pow(C,2))); 
    fls = Math.sqrt(Math.pow(B,2)+Math.pow(D,2));
    rrs = Math.sqrt(Math.pow(A,2)+Math.pow(D,2)); 
    rls = Math.sqrt(Math.pow(A,2)+Math.pow(C,2)); 
         
    fra = Math.atan2(B,C)*180/Math.PI;
    fla = Math.atan2(B,D)*180/Math.PI;
    rra = Math.atan2(A,D)*180/Math.PI;
    rla = Math.atan2(A,C)*180/Math.PI;
      
    frSteer.set((fra+180) * (1023/360));
    flSteer.set((fla+180) * (1023/360));
    rlSteer.set((rla+180) * (1023/360));
    rrSteer.set((rra+180) * (1023/360));

    max = frs;

    if(fls>max){
    	max = fls;
    }
    if(rls>max){
    	max = rls;
    }
    if(rrs>max){
    	max = rrs;
    }
    if(max>1){
    	frs/=max;
    	fls/=max;
    	rrs/=max;
    	rls/=max;
    }

    frDrive.set(frs*12);
    flDrive.set(fls*12);
    rrDrive.set(rrs*12);
    rlDrive.set(rls*12);
    }

}
// CANTalon FLdrive, FLsteer, RLdrive, RLsteer, FRdrive, FRsteer, RRdrive, RRsteer;
//   Joystick driveJoy;
//   Gyro gyro;

//   public Robot() {
//       FLdrive = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
//       FLsteer = new CANTalon(2); 
//       RLdrive = new CANTalon(3); 
//       RLsteer = new CANTalon(4);
//       FRdrive = new CANTalon(5);
//       FRsteer = new CANTalon(6);
//       RRdrive = new CANTalon(7);
//       RRsteer = new CANTalon(8);
//       driveJoy = new Joystick(1);

//       FLdrive.changeControlMode(CANTalon.ControlMode.Voltage);
//       RLdrive.changeControlMode(CANTalon.ControlMode.Voltage);
//       FRdrive.changeControlMode(CANTalon.ControlMode.Voltage);
//       RRdrive.changeControlMode(CANTalon.ControlMode.Voltage);
//       FLsteer.changeControlMode(CANTalon.ControlMode.Position);
//       RLsteer.changeControlMode(CANTalon.ControlMode.Position);
//       FRsteer.changeControlMode(CANTalon.ControlMode.Position);
//       RRsteer.changeControlMode(CANTalon.ControlMode.Position);

//       FLsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
//       RLsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
//       FRsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
//       RRsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
//       FLdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
//       RLdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
//       FRdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
//       RRdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

//       FLsteer.setPID(1, 0, 0);
//       RLsteer.setPID(1, 0, 0);
//       FRsteer.setPID(1, 0, 0);
//       RRsteer.setPID(1, 0, 0);
//   }

//   public void operatorControl() {
//     while (isOperatorControl() && isEnabled()) {

//     	double fwd;
//     	double str;
//     	double rcw;
//     	double temp;
//     	double fwd2;
//     	double str2;
//     	double wheelbase;
//     	double trackwidth;
//     	double r;
//     	double a;
//     	double b;
//     	double c;
//     	double d;
//     	double frs, fls, rls, rrs; //Front Right, Front Left, Rear Left, Rear Right Wheel Speeds, respectively
//     	double fra, fla, rla, rra; //Wheel Angles
//     	double max;
//     	fwd = driveJoy.getY();
//     	str = driveJoy.getX();
//     	rcw = driveJoy.getZ();
    	
//     	temp = (fwd*Math.cos(gyro.getAngle())) + str*Math.sin(gyro.getAngle());
//     	str2 = (-fwd*Math.sin(gyro.getAngle())) + str*Math.cos(gyro.getAngle());
//     	fwd2 = temp;
    	
//     	wheelbase = 30; //length of drivebase
//     	trackwidth = 24; //width of drivebase
//     	r = Math.sqrt((wheelbase*wheelbase) + (trackwidth*trackwidth));
    	
//     	a = str2 - rcw * (wheelbase/r);
//     	b = str2 + rcw * (wheelbase/r);
//     	c = fwd2 - rcw * (trackwidth/r);
//     	d = fwd2 + rcw * (trackwidth/r);
    	
//     	frs = Math.sqrt(b*b + c*c);
//     	fls = Math.sqrt(b*b + d*d);
//     	rls = Math.sqrt(a*a + d*d);
//     	rrs = Math.sqrt(a*a + c*c);
    	
//     	fra = Math.atan2(b,c) * 180/Math.PI;
//     	fla = Math.atan2(b,d) * 180/Math.PI;
//     	rra = Math.atan2(a,d) * 180/Math.PI;
//     	rla = Math.atan2(a,c) * 180/Math.PI;
    	
//     	//Because the CANTalon position control uses values from 0 - 1023 
//     	//for potentiometer ranges, we must modify the wheel angles to 
//     	//compenstate for this. To do this add 180 to the wheel angle to 
//     	//get a 0-360 range then multiply by 1023/360 which equals 2.8444...
    	
//     	FRsteer.set((fra+180) * (1023/360));
//     	FLsteer.set((fla+180) * (1023/360));
//     	RLsteer.set((rla+180) * (1023/360));
//     	RRsteer.set((rra+180) * (1023/360));
    	
//     	//Normalize wheel speeds as stated in Ether's document
//     	max = frs;
//     	if(fls>max){
//     		max = fls;
//     	}
//     	if(rls>max){
//     		max = rls;
//     	}
//     	if(rrs>max){
//     		max = rrs;
//     	}
//     	if(max>1){
//     		frs/=max;
//     		fls/=max;
//     		rrs/=max;
//     		rls/=max;
//     	}
//     	//Wheel speeds are now 0-1. Not -1 to +1.
//     	//Multiply wheel speeds by 12 for voltage control mode
//     	FRdrive.set(frs*12);
//     	FLdrive.set(fls*12);
//     	RRdrive.set(rrs*12);
//     	RLdrive.set(rls*12);
//     }
//     }
//   }