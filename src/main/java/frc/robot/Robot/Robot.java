package frc.robot.Robot;

//Utilizes all mechanisms for the robot and runs their programs to perform each function


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


//import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.net.PortForwarder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

//Below Imports are from the PhantomBanana
import frc.robot.Autonomous.BananaDriveStraight;
import frc.robot.Autonomous.BananaTurn;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.BananaArm;
import frc.robot.Mechanisms.BananaBrake;
import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Mechanisms.BananaBrake;
import frc.robot.Mechanisms.BananaClaw;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static BananaArm         arm;
  public static BananaDriveTrain  driveTrain;
  public static BananaBrake      brake;
  public static BananaClaw        claw;

  //private static XboxController xboxDrv;
  private static XboxController controller ;

  //private static final int XBOX_DRV_PORT = 0;
  private static final int XBOX_DRV_PORT = 0;

  private Thread aimPID;

  private float left_command;
  private float right_command;

  @Override
  public void robotInit() 
  {

    /*--------------------------------------------------------------------------
    *  Initialize Drive Cameras
    *-------------------------------------------------------------------------*/
   PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
   
   
    /*--------------------------------------------------------------------------
    *  Initialize Mechanisms & Drive Controllers
    *-------------------------------------------------------------------------*/
    arm        = new BananaArm();
    driveTrain = new BananaDriveTrain();
    claw = new BananaClaw();
    brake       = new BananaBrake();
  
    //xboxDrv    = new XboxController(XBOX_DRV_PORT);
    controller    = new XboxController(XBOX_DRV_PORT);

    
    
    /*--------------------------------------------------------------------------
    *  Initialize Vision
    *-------------------------------------------------------------------------*/
    //might not need to

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    
    
    arm.setPivotTargetAngle(arm.getPivotAngle());
    arm.pivotPID();

    
  }


  @Override
  public void robotPeriodic() 
  {
         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", x); //(x,y) from the crosshair on the camera stream in pixle units
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area); //area of FOV that the target takes up
    SmartDashboard.putNumber("Limelight Valid Target", v);//0 for no valid target, 1 for valid target


    //DriveBase
    SmartDashboard.putNumber("FR Motor Temperature", driveTrain.getMotorTemperature(27));
    SmartDashboard.putNumber("BR Motor Temperature", driveTrain.getMotorTemperature(28));
    SmartDashboard.putNumber("FL Motor Temperature", driveTrain.getMotorTemperature(26));
    SmartDashboard.putNumber("BL Motor Temperature", driveTrain.getMotorTemperature(25));
    
    //Arm          
    SmartDashboard.putNumber("PIVOT: Target Angle", arm.getPivotTargetAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", BananaArm.armPivotEnc.getVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());
    // TODO SmartDashboard.putNumber("pivot power", arm.getPivotPower());
    
   

    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;
  
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 35.5;//WILL CHANGE THIS BECAUSE WE ARE USING A NEW ROBOT
  
    // distance from the target to the floor
    double goalHeightInches = 104.0;//WILL CHANGE BECAUSE THE GOALS ARE NOW DIFFERENT (MIGHT NOT EVEN NEED HEIGHT SINCE WE WORK IN INCHES)
  
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;//AGAIN WE MAY NOT NEED THIS
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);//DOUBLE CHECK IF WE ARE GOING TO BE DOING IT THIS WAY)
  
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians); //(COULD POTENTIALLY USE THIS TO TELL
                                                                                                                            //THE ROBOT WHEN TO STOP IN FRONT OF
                                                                                                                            //THE GOAL TO BEGIN RAISING THE ARM,
                                                                                                                            //THEN TO MOVE CLOSER TO SCORE THE GOAL
  }

  @Override
  public void autonomousInit() 
  {
    initializeRobotPositions();
  }

  @Override
  public void autonomousPeriodic()
  {
    robotControls();
  }

  @Override
  public void teleopInit() 
  {
    initializeRobotPositions();

  }

  @Override
  public void teleopPeriodic() 
  {
    robotControls();
    
  }


  private void initializeRobotPositions()
  {
      arm.setPivotTargetAngle(arm.getPivotAngle());
  }

  private void robotControls()
  { 
    
    /*--------------------------------------------------------------------------
    *  DriveBase controls
    *-------------------------------------------------------------------------*/
    
    driveTrain.tankDrive(controller.getLeftY(), controller.getRightY());
   
     /*--------------------------------------------------------------------------
    *  DriveBase shifter
    *-------------------------------------------------------------------------*/
     
    aimPID = new Thread(() ->
    {
      float kAimP = -0.1f;  //may need to calibrate kAimP or min_command if aiming causes occilation
      float min_command = 0.05f;
  
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tx = table.getEntry("tx");
      float x = tx.getFloat(0.0f);

  
      if (controller.getLeftBumperPressed())
      {
          float heading_error = -x;
          float steering_adjust = 0.0f;
  
          if (Math.abs(heading_error) > 1.0)
          {
                  if (heading_error < 0)
                  {
                          steering_adjust = kAimP*heading_error + min_command;
                  }
                  else
                  {
                          steering_adjust = kAimP*heading_error - min_command;
                  }
          }
          left_command += steering_adjust;
          right_command -= steering_adjust;
      }

      
    });
        
      
  
       
     
     
     
     /*--------------------------------------------------------------------------
    *  Pivot Movement - Manual Control
    *-------------------------------------------------------------------------*/
    
    
   
      /*--------------------------------------------------------------------------
    *  Arm Movement - Manual Control
    *-------------------------------------------------------------------------*/
      
    
      
      
    
       /*-----------------------------------------------------------------------
       *  Out of Deadband - Manual Control
       *----------------------------------------------------------------------*/
       
       /*-----------------------------------------------------------------------
       *  In Deadband - Hold Position
       *----------------------------------------------------------------------*/
      
         // only stop arm if pid thread is not running 
         
         
     /*--------------------------------------------------------------------------
    *  Drive Controller - Presets
    *-------------------------------------------------------------------------*/
     
     
   /*boolean parkingBreak = false;
    if(controller.getLeftBumper()){
        parkingBreak = !parkingBreak;
    }
    */

    /*--------------------------------------------------------------------------
    *  Aux Controller - Pick Up Position Presets
                        Preset Arm Positions
    *-------------------------------------------------------------------------*/
     
    if(controller.getStartButton())
    {
      BananaPreSets.neutralPivotAngle();
    }
    if(controller.getAButton())
    {
      BananaPreSets.cargoPickUp();
    }
    if(controller.getBackButton())
    {
      BananaPreSets.hatchPickUp();
    }

     /*--------------------------------------------------------------------------
    *  Aux Controller - Scoring Position Presets
    *                   Preset Arm Positions
    *-------------------------------------------------------------------------*/
     
    if(controller.getBButton())
    {
      BananaPreSets.lvl3RocketBall();
    }
    if(controller.getYButton())
    {
      BananaPreSets.lvl2RocketBall();
    }
    if(controller.getXButton())
    {
      BananaPreSets.lvl1RocketBall();
    }
    
    
  }

}
