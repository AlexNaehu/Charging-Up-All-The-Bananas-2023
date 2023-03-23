package frc.robot.Autonomous;


import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Mechanisms.BananaIntake;
import frc.robot.Robot.BananaPreSets;
import frc.robot.Robot.Robot;

public class BananaAutonPaths {





    





    private final static double leftHalfTurn = 90.0; //"left turn" means the front right corner whips around CCW
    private final static double leftFullTurn = 180; // CHANGE TO ADJUST ROBOT TO FACE CENTER PERPENDICULARLY AFTER FACING THE SCORING AT AN ANGLE
    private final static double rightHalfTurn = -90.0; //"right turn" means the front left corner whips around CW
    private final static double rightFullTurn = -180; // CHANGE TO ADJUST ROBOT TO FACE CENTER PERPENDICULARLY AFTER FACING THE SCORING AT AN ANGLE

    

    private final static int straightForwardSide = 0;
    private final static int straightForwardMid = 1;
    private final static int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final static int straightBackwardSide = 3;
    private final static int straightBackwardMid = 4;
    private final static int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring

    private final static int rightTurn = 6;
    private final static int leftTurn = 7;
    private final static int hyperRightTurn = 8;
    private final static int hyperLeftTurn = 9;
    private static double currentTime;

    /*IMPORTANT: FIGURE OUT HOW TO PRELOAD OBJECTS INTO THE Intake*/
    /*Also Important: Consider using a timer to seperate each command by 0.2 seconds so that error doesn't stack */


    //TEST HOW LONG IT TAKES TO CHANGE BETWEEN CERTAIN ANGLES
    //TEST HOW LONG IT TAKES TO OPEN AND CLOSE THE Intake
    //TEST HOW LONG IT TAKES TO 
    

   






    public static void rightScoreMob() //turns left, away from the wall on the right side of the robot
    {   

        currentTime = Robot.autonClock.get();
        


        //comment out everything and uncomment each line after testing each time
 
        //if(currentTime>0.0 && currentTime<0.5)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=0.5 && currentTime<1.1)
        //BananaPreSets.lvl3RocketBall();
        //if(currentTime>=1.15 && currentTime<2.3)//1.5S
        //BananaDriveStraight.driveStraight(tipToeForward);
        //if(currentTime>=2.35 && currentTime<3.20)
        //BananaIntake.output(1);
        //if(currentTime>=3.2 && currentTime<3.25)
        //BananaIntake.output(0);
        //if(currentTime>=3.30 && currentTime<4.80)//1.5S
        //BananaDriveStraight.driveStraight(tipToeBackward);
        //if (currentTime>=4.80 && currentTime<5.30)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=5.35 && currentTime<5.85)
        //BananaPreSets.cargoPickUp();
        if(currentTime>=5.5 && currentTime<6.25){
            BananaDriveStraight.driveStraight(hyperRightTurn);
            BananaPreSets.cargoPickUp();}
        //if(currentTime>=6.30 && currentTime<9.20)
        //BananaDriveStraight.driveStraight(straightForwardSide); 
        ////Robot.driveTrain.aimPIDState = true;
        //if(currentTime>=8.80 && currentTime<9.80) //giving 0.5 second for Intake to close
        //BananaIntake.intake(1);
        //if(currentTime>=9.80 && currentTime<9.85)
        //BananaIntake.intake(0);
        ////Robot.driveTrain.aimPIDState = false;
        //if(currentTime>=9.90 && currentTime<10.90)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=10.47 && currentTime<11.30){
        //BananaDriveStraight.driveStraight(hyperRightTurn);
        //BananaPreSets.lvl1RocketBall();} 
        //if(currentTime>=11.35 && currentTime<14.45) //has to travel far enough to place the claw above the lvl2 shelf
        //BananaDriveStraight.driveStraight(straightForwardSide); 
        //if(currentTime>=13.25 && currentTime<14.25) //raises arm while driving back towards goal
        //BananaPreSets.lvl2RocketBall();
        //if(currentTime>=14.50 && currentTime<14.95)
        //BananaIntake.output(1);
        //if(currentTime>=14.95 && currentTime<15.0)
        //BananaIntake.output(0);

        //END OF ROLLER AUTON



        /*
        //START OF CLAW AUTON

        currentTime = Robot.autonClock.get();
        
        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=1.0 && currentTime<2.50)
        BananaDriveStraight.driveStraight(tipToeBackward); 
        if(currentTime>=2.45 && currentTime<3.30)
        BananaPreSets.cargoPickUp();
        if(currentTime>=3.30 && currentTime<3.60)
        BananaDriveStraight.driveStraight(tipToeForward);
        if(currentTime>=3.30 && currentTime<4.35)
        BananaIntake.intake(0.5); 
        if (currentTime>=4.35 && currentTime<4.40)
        BananaIntake.intake(0);
        if(currentTime>=4.45 && currentTime<5.35)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=5.40 && currentTime<6.30)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=6.35 && currentTime<9.25)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        BananaDriveStraight.driveStraight(tipToeForward); 
        ////Robot.driveTrain.aimPIDState = true;
        if(currentTime>=9.30 && currentTime<9.80) //giving 0.5 second for Intake to open
        BananaIntake.output(0.5);
        if(currentTime>=9.80 && currentTime<9.85)
        BananaIntake.output(0);
        ////Robot.driveTrain.aimPIDState = false;
        if(currentTime>=9.90 && currentTime<11.40)
        BananaDriveStraight.driveStraight(tipToeBackward); 
        if(currentTime>=11.45 && currentTime<12.05)
        BananaPreSets.lvl2RocketBall();
        if(currentTime>=12.10 && currentTime<13.0)
        BananaPreSets.cargoPickUp();
        if(currentTime>=13.05 && currentTime<13.4) //+0.6 extra to complete the 180 turn
        BananaDriveStraight.driveStraight(hyperRightTurn);
        if(currentTime>=13.4 && currentTime<14.5)
        BananaDriveStraight.driveStraight(straightForwardSide);
        if(currentTime>=14.55 && currentTime<15.0)
        BananaIntake.intake(0.5);

        //END OF CLAW AUTON
        */
        
        
       
        

    }

    public static void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        
        currentTime = Robot.autonClock.get();
        


        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.0 && currentTime<0.5)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=0.5 && currentTime<1.1)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=1.15 && currentTime<2.65)//1.5S
        BananaDriveStraight.driveStraight(tipToeForward);
        if(currentTime>=2.70 && currentTime<3.20)
        BananaIntake.output(0.5);
        if(currentTime>=3.2 && currentTime<3.25)
        BananaIntake.output(0);
        if(currentTime>=3.30 && currentTime<4.80)//1.5S
        BananaDriveStraight.driveStraight(tipToeBackward);
        if (currentTime>=4.80 && currentTime<5.30)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=5.35 && currentTime<5.85)
        BananaPreSets.cargoPickUp();
        if(currentTime>=5.42 && currentTime<6.25){
            BananaDriveStraight.driveStraight(hyperLeftTurn);
            BananaPreSets.cargoPickUp();}
        if(currentTime>=6.30 && currentTime<9.20)
        BananaDriveStraight.driveStraight(straightForwardSide); 
        ////Robot.driveTrain.aimPIDState = true;
        if(currentTime>=8.80 && currentTime<9.80) //giving 0.5 second for Intake to close
        BananaIntake.intake(0.5);
        if(currentTime>=9.80 && currentTime<9.85)
        BananaIntake.intake(0);
        ////Robot.driveTrain.aimPIDState = false;
        if(currentTime>=9.90 && currentTime<10.90)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=10.47 && currentTime<11.30){
        BananaDriveStraight.driveStraight(hyperLeftTurn);
        BananaPreSets.lvl1RocketBall();}  
        if(currentTime>=11.35 && currentTime<14.45) //has to travel far enough to place the claw above the lvl2 shelf
        BananaDriveStraight.driveStraight(straightForwardSide); 
        if(currentTime>=13.25 && currentTime<14.25) //raises arm while driving back towards goal
        BananaPreSets.lvl2RocketBall();
        if(currentTime>=14.50 && currentTime<14.95)
        BananaIntake.output(0.5);
        if(currentTime>=14.95 && currentTime<15.0)
        BananaIntake.output(0);

        //END OF ROLLER AUTON



        /*
        //START OF CLAW AUTON

        currentTime = Robot.autonClock.get();
        
        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=1.0 && currentTime<2.50)
        BananaDriveStraight.driveStraight(tipToeBackward); 
        if(currentTime>=2.45 && currentTime<3.30)
        BananaPreSets.cargoPickUp();
        if(currentTime>=3.30 && currentTime<3.60)
        BananaDriveStraight.driveStraight(tipToeForward);
        if(currentTime>=3.30 && currentTime<4.35)
        BananaIntake.intake(0.5); 
        if (currentTime>=4.35 && currentTime<4.40)
        BananaIntake.intake(0);
        if(currentTime>=4.45 && currentTime<5.35)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=5.40 && currentTime<6.30)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=6.35 && currentTime<9.25)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        BananaDriveStraight.driveStraight(tipToeForward); 
        ////Robot.driveTrain.aimPIDState = true;
        if(currentTime>=9.30 && currentTime<9.80) //giving 0.5 second for Intake to open
        BananaIntake.output(0.5);
        if(currentTime>=9.80 && currentTime<9.85)
        BananaIntake.output(0);
        ////Robot.driveTrain.aimPIDState = false;
        if(currentTime>=9.90 && currentTime<11.40)
        BananaDriveStraight.driveStraight(tipToeBackward); 
        if(currentTime>=11.45 && currentTime<12.05)
        BananaPreSets.lvl2RocketBall();
        if(currentTime>=12.10 && currentTime<13.0)
        BananaPreSets.cargoPickUp();
        if(currentTime>=13.05 && currentTime<13.4) //+0.6 extra to complete the 180 turn
        BananaDriveStraight.driveStraight(hyperLeftTurn);
        if(currentTime>=13.4 && currentTime<14.5)
        BananaDriveStraight.driveStraight(straightForwardSide);
        if(currentTime>=14.55 && currentTime<15.0)
        BananaIntake.intake(0.5);

        //END OF CLAW AUTON
        */
    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public static void midScorePark() 
    {
        
        currentTime = Robot.autonClock.get();
        


        //comment out everything and uncomment each line after testing each time
 
        //if(currentTime>0.0 && currentTime<0.5)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=0.5 && currentTime<1.1)
        //BananaPreSets.lvl3RocketBall();
        //if(currentTime>=1.15 && currentTime<2.65)//1.5S
        //BananaDriveStraight.driveStraight(tipToeForward);
        //if(currentTime>=2.70 && currentTime<3.20)
        //BananaIntake.output(0.5);
        //if(currentTime>=3.2 && currentTime<3.25)
        //BananaIntake.output(0);
        //if(currentTime>=3.30 && currentTime<4.80)//1.5S
        //BananaDriveStraight.driveStraight(tipToeBackward);
        //if (currentTime>=4.80 && currentTime<5.30)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=5.35 && currentTime<5.85)
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=5.42 && currentTime<6.25){
        //BananaDriveStraight.driveStraight(hyperLeftTurn);
        //BananaPreSets.cargoPickUp();}
        //if(currentTime>=6.30 && currentTime<9.20)
        //BananaDriveStraight.driveStraight(straightForwardSide); 
        if(currentTime>=9.25 && currentTime<15.0){
        BananaDriveTrain.balance(Robot.navx.getRoll());
        }
        







        /* 
        currentTime = Robot.autonClock.get();
        
        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=1.0 && currentTime<2.50)
        BananaDriveStraight.driveStraight(tipToeBackward);
        if(currentTime>=2.45 && currentTime<3.30)
        BananaPreSets.cargoPickUp();
        if(currentTime>=3.30 && currentTime<3.60)
        BananaDriveStraight.driveStraight(tipToeForward);
        if(currentTime>=3.30 && currentTime<4.35)
        BananaIntake.intake(0.5);
        if (currentTime>=4.35 && currentTime<4.40)
        BananaIntake.intake(0);
        if(currentTime>=4.45 && currentTime<5.35)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=5.40 && currentTime<6.30)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=6.35 && currentTime<9.25)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        BananaDriveStraight.driveStraight(tipToeForward);
        ////Robot.driveTrain.aimPIDState = true;
        if(currentTime>=9.30 && currentTime<9.80) //giving 0.6 second for Intake to open
        BananaIntake.output(0.5);
        if(currentTime>=9.80 && currentTime<9.85)
        BananaIntake.output(0);
        ////Robot.driveTrain.aimPIDState = false;
        if(currentTime>=9.90 && currentTime<11.40)
        BananaDriveStraight.driveStraight(tipToeBackward);
        if(currentTime>=11.45 && currentTime<12.05)
        BananaPreSets.lvl2RocketBall();
        if(currentTime>=12.10 && currentTime<13.0)
        BananaPreSets.cargoPickUp();
        if(currentTime>=13.05 && currentTime<13.4) //+0.6 extra to complete the 180 turn
        BananaDriveStraight.driveStraight(hyperRightTurn);
        if(currentTime>=13.4 && currentTime<14.0) //need to trial and error to get the robot to drive right onto the center of the platform
        BananaDriveStraight.driveStraight(straightForwardSide);
        */

    } 
}
