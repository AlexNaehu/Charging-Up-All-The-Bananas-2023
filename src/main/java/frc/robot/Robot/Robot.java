package frc.robot.Robot;

//Utilizes all mechanisms for the robot and runs their programs to perform each function


//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.hal.ThreadsJNI;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.net.PortForwarder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


//import frc.robot.Autonomous.BananaDriveStraight;
//import frc.robot.Autonomous.BananaTurn;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autonomous.BananaAutonPaths;
import frc.robot.Mechanisms.BananaArm;
import frc.robot.Mechanisms.BananaBrake;
import frc.robot.Mechanisms.BananaIntake;
import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Vision.SensorObject;


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
  public static BananaIntake        Intake;
  public static SensorObject    sensor;

  private UsbCamera bananaCam;

  //private static final double LEFT_DEADBAND_THRESHOLD = 0.15;
  //private static final double RIGHT_DEADBAND_THRESHOLD = 0.15;
  double pThr = 0.0;

  //private boolean armPIDState = false;

  private boolean gear1 = false;//0.3 Power
  private boolean gear2 = true;//0.5 Power
  private boolean gear3 = false;//0.8 Power

  public static AHRS navx;
  
  public static XboxController controller1;
  public static XboxController controller2;

  
  private static final int XBOX_PORT_1 = 0;
  private static final int XBOX_PORT_2 = 1;

  private static final String LeftScoreMob = "LeftScoreMob";
  private static final String MidScorePark = "MidScorePark";
  private static final String RightScoreMob = "RightScoreMob";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static Timer autonClock;

  //private DifferentialDriveOdometry odometry;

  @Override
  public void robotInit() 
  {

    /*--------------------------------------------------------------------------
    *  Initialize Drive Cameras
    *-------------------------------------------------------------------------*/

    bananaCam = CameraServer.startAutomaticCapture();
    bananaCam.setFPS(15);
    bananaCam.setResolution(320, 240);
    bananaCam.setPixelFormat(PixelFormat.kMJPEG);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
   
   
    /*--------------------------------------------------------------------------
    *  Initialize Mechanisms & Drive Controllers
    *-------------------------------------------------------------------------*/
    arm        = new BananaArm();
    driveTrain = new BananaDriveTrain();
    Intake = new BananaIntake();
    brake       = new BananaBrake();
  
    sensor = new SensorObject();

    navx = new AHRS();

    controller1    = new XboxController(XBOX_PORT_1);
    controller2    = new XboxController(XBOX_PORT_2);

    //odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), new Pose2d(5.0, 13.5, new Rotation2d()));
    
    /*--------------------------------------------------------------------------
    *  Initialize Auton
    *-------------------------------------------------------------------------*/
    m_chooser.setDefaultOption("RightScoreMob", RightScoreMob);
    m_chooser.addOption("MidScorePark", MidScorePark);
    m_chooser.addOption("LeftScoreMob", LeftScoreMob);
   
    SmartDashboard.putData("Auto choices", m_chooser);

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    
    navx.reset();

    arm.setPivotTargetAngle(arm.getPivotAngle());
    
    arm.pivotPID();
    
                                                                                                                                                                                                                                                                                                                                                                                       
    driveTrain.coneAimPID();
    //driveTrain.cubeAimPID(); //prolly not going to use this bc of USB load on roboRio CPU
    //sensor.sensorObject(); //def cant use bc of image processing load on roboRio load


    var timer = new Timer();
    timer.start();

    autonClock = new Timer(); //starts in autonInit()
    
  }


  @Override
  public void robotPeriodic() 
  {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableInstance table = NetworkTableInstance.getDefault();

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
    
    //Limelight
    SmartDashboard.putNumber("Limelight X", x); //(x,y) from the crosshair on the camera stream in pixle units
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area); //area of FOV that the target takes up
    SmartDashboard.putNumber("Limelight Valid Target", v);//0 for no valid target, 1 for valid target

    //USB Camera
    bananaCam.setFPS(15);
    bananaCam.setResolution(320, 240);
    
    //Sensors
    SmartDashboard.putNumber("Robot Orientation", navx.getAngle());
   
    
    
    //DriveBase
    SmartDashboard.putBoolean("Cone Aim PID State", BananaDriveTrain.coneAimPIDState);
    SmartDashboard.putBoolean("Cube Aim PID State", BananaDriveTrain.cubeAimPIDState);
    SmartDashboard.putNumber("FR Motor Temperature", driveTrain.getMotorTemperature(21));
    SmartDashboard.putNumber("BR Motor Temperature", driveTrain.getMotorTemperature(22));
    SmartDashboard.putNumber("FL Motor Temperature", driveTrain.getMotorTemperature(20));
    SmartDashboard.putNumber("BL Motor Temperature", driveTrain.getMotorTemperature(23));
    SmartDashboard.putNumber("Cone Left Command", driveTrain.cone_left_command);
    SmartDashboard.putNumber("Cone Right Command", driveTrain.cone_right_command);
    SmartDashboard.putNumber("Cube Left Command", driveTrain.cube_left_command);
    SmartDashboard.putNumber("Cube Right Command", driveTrain.cube_right_command);
    
    
    //Arm          

    
    //SmartDashboard.putBoolean("Arm Target Hit", arm.armTargetHit);
    SmartDashboard.putNumber("PIVOT: Target Angle", arm.getPivotTargetAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.armPivotEnc.getVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());
    SmartDashboard.putNumber("Right Arm Angler Temperature", arm.getArmTemp(24));
    SmartDashboard.putNumber("Left Arm Angler Temperature", arm.getArmTemp(28));
    

    //Intake
    SmartDashboard.putBoolean("Intake: Open State", Intake.isIntakeOpen());
    SmartDashboard.putNumber("Intake: Left Power", Intake.getLeftRollerPower());
    SmartDashboard.putNumber("Intake: Right Power", Intake.getRightRollerPower());

    //Brake
    SmartDashboard.putBoolean("Brake: On State", brake.isBrakeOn());
    SmartDashboard.putNumber("Brake: Left Power", brake.getLeftBrakePower());
    SmartDashboard.putNumber("Brake: Right Power", brake.getRightBrakePower());



    //double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    //double limelightMountAngleDegrees = 25.0;
  





    // distance from the center of the Limelight lens to the floor
    //double limelightLensHeightInches = 35.5;//WILL CHANGE THIS BECAUSE WE ARE USING A NEW ROBOT
  
    // distance from the target to the floor
    //double goalHeightInches = 104.0;//WILL CHANGE BECAUSE THE GOALS ARE NOW DIFFERENT (MIGHT NOT EVEN NEED HEIGHT SINCE WE WORK IN INCHES)
  
    //double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;//AGAIN WE MAY NOT NEED THIS
    //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);//DOUBLE CHECK IF WE ARE GOING TO BE DOING IT THIS WAY)
  
    //calculate distance
    //double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians); //(COULD POTENTIALLY USE THIS TO TELL
                                                                                                                            //THE ROBOT WHEN TO STOP IN FRONT OF
                                                                                                                            //THE GOAL TO BEGIN RAISING THE ARM,
                                                                                                                            //THEN TO MOVE CLOSER TO SCORE THE GOAL
  }

  @Override
  public void autonomousInit() 
  {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    initializeRobotPositions();

    autonClock.reset();
    autonClock.start();

  }

  @Override
  public void autonomousPeriodic() //periodically calls one of three battle plans >:D
  {
    
    //Moved to autonomousInit();
    //autonClock.reset();
    //autonClock.start();

    switch (m_autoSelected) {

      case LeftScoreMob:
      //BananaAutonPaths.leftScoreMob();
        // Put custom auto code here
        break;
      case MidScorePark:
      //BananaAutonPaths.midScorePark();
        // Put custom auto code here
        break;
      case RightScoreMob:
        BananaAutonPaths.rightScoreMob();
        // Put default auto code here
        break;
    }
    

    
  }
    
  @Override
  public void teleopInit() 
  {
    autonClock.stop();
    initializeRobotPositions();

  }

  @Override
  public void teleopPeriodic() 
  {
    
    robotControls();
    
  }


  private void robotControls()
  {

    /*--------------------------------------------------------------------------
    *  DriveBase Movement - Manual Control (1)
    *-------------------------------------------------------------------------*/
    
    if (controller1.getAButtonPressed()) //0.3 Speed
    {
    gear1 = true;
    gear2 = false;
    gear3 = false;
    }

    if (controller1.getXButtonPressed()) //0.5 Speed
    {
      
    gear1 = false;
    gear2 = true;
    gear3 = false;
    }

    if (controller1.getYButtonPressed()) //0.7 Speed
    {
      gear1 = false;
      gear2 = false;
      gear3 = true;
    }
     

    if (gear1)
    {
    driveTrain.tankDriveLow(controller1.getLeftY(), controller1.getRightY()); // LEFT + RIGHT STICKS // 1
    }

    if (gear2)
    {
    driveTrain.tankDriveMid(controller1.getLeftY(), controller1.getRightY()); // LEFT + RIGHT STICKS // 1
    }

    if(gear3)
    {
    driveTrain.tankDriveHigh(controller1.getLeftY(), controller1.getRightY()); // LEFT + RIGHT STICKS // 1
    }
    
    //3 Levels of SPEEEED
    


    /*--------------------------------------------------------------------------
    *  DriveBase Movement - AimBot (1)
    *-------------------------------------------------------------------------*/
    
    if(controller1.getStartButtonPressed()) // START BUTTON // 1
    {
      BananaDriveTrain.coneAimPIDState = !(BananaDriveTrain.coneAimPIDState);
    }

    if(controller1.getBackButtonPressed()) // BACK Button // 1
    {
      BananaDriveTrain.cubeAimPIDState = !(BananaDriveTrain.cubeAimPIDState);
    }
 
    /*--------------------------------------------------------------------------
    *  Intake Movement - Manual Control (1)
    *-------------------------------------------------------------------------*/
    
    if(controller1.getLeftTriggerAxis()<0.05 && controller1.getRightTriggerAxis()>0.05)
    {
      BananaIntake.intake(controller1.getRightTriggerAxis()); // RIGHT TRIGGER // 1
    }
    else if (controller1.getLeftTriggerAxis() > 0.05 && controller1.getRightTriggerAxis() < 0.05)
    {
      BananaIntake.output(controller1.getLeftTriggerAxis()); // LEFT TRIGGER // 1
    }
    else
    {
      BananaIntake.intake(0.0); //Stop Motor
    }
    
    /*--------------------------------------------------------------------------
    *  Arm Movement - Manual Control (1)
    *-------------------------------------------------------------------------*/

    if (controller1.getRightBumperPressed()) // RIGHT BUMPER // 1 // Maybe do getRightBumper()
    {
      arm.increaseTargetAngle();
      //BananaArm.testMotorsUp();
    }

    /*
     * if(controller1.getRightBumper())
    {
      arm.increaseTargetAngle();
    }
    else if (controller1.getLeftBumper())
    {
      arm.decreaseTargetAngle();
    }
    else
    {
      BananaIntake.closeIntake(0.0); //Stop Motor
    }
     */

    if (controller1.getLeftBumperPressed()) // LEFT BUMPER // 1
    {
      arm.decreaseTargetAngle();
      //BananaArm.testMotorsDown();
    }
    

    /*--------------------------------------------------------------------------
    *  Brake Movement - Presets (2)
    *-------------------------------------------------------------------------*/
     
    

    /*-----------------------------------------------------------------------
    *  Out of Deadband - Manual Control (2)
    *----------------------------------------------------------------------*/
       
       /* 
     if (controller2.getRightTriggerAxis()>RIGHT_DEADBAND_THRESHOLD)
      pThr = controller2.getRightTriggerAxis();
    if (controller2.getLeftTriggerAxis()>LEFT_DEADBAND_THRESHOLD)
      pThr = (controller2.getLeftTriggerAxis()*0.2);

    if(Math.abs(pThr) > LEFT_DEADBAND_THRESHOLD)
    {
      armPIDState = false;
      arm.setPivotTargetAngle(BananaConstants.INVALID_ANGLE);
      arm.pivotArm(pThr);
    }
    else 
    {
      if(armPIDState == false)
      {
        arm.setPivotTargetAngle(arm.getPivotAngle());
        armPIDState = true;
      } 
    }
    
      */
    
        
    /*--------------------------------------------------------------------------
    *  Aux Controller - Preset Arm Positions (2)
    *-------------------------------------------------------------------------*/
      
    if(controller2.getStartButton()) // START BUTTON // 2
    {
      BananaPreSets.neutralPivotAngle();
    }

    if(controller2.getRightBumper()) // BACK BUTTON // 2
    {
      BananaPreSets.hatchPickUp();
    }

    if(controller2.getAButton()) // A BUTTON // 2
    {
      BananaPreSets.cargoPickUp();
    }

    /*--------------------------------------------------------------------------
    *  Aux Controller - Preset Scoring Positions (2)
    *-------------------------------------------------------------------------*/
     
    if(controller2.getYButton())
    {
      BananaPreSets.lvl3RocketBall(); // Y BUTTON // 2
    }

    if(controller2.getXButton())
    {
      BananaPreSets.lvl2RocketBall(); // X BUTTON // 2
    }

    if(controller2.getBButton())
    {
      BananaPreSets.lvl1RocketBall(); // B BUTTON // 2
    }

    
    

  } //End of robot controlls



  private void initializeRobotPositions()
  {
    arm.setPivotTargetAngle(arm.getPivotAngle());

  }

  

}
