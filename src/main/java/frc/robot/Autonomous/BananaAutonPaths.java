package frc.robot.Autonomous;


import frc.robot.Mechanisms.BananaClaw;
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

    /*IMPORTANT: FIGURE OUT HOW TO PRELOAD OBJECTS INTO THE CLAW*/
    /*Also Important: Consider using a timer to seperate each command by 0.2 seconds so that error doesn't stack */


    //TEST HOW LONG IT TAKES TO CHANGE BETWEEN CERTAIN ANGLES
    //TEST HOW LONG IT TAKES TO OPEN AND CLOSE THE CLAW
    //TEST HOW LONG IT TAKES TO 
    

   






    public static void rightScoreMob() //turns left, away from the wall on the right side of the robot
    {   

        
        currentTime = Robot.autonClock.get();
        


        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();//cargoPickup();
        if(currentTime>=1.0 && currentTime<2.50)//2.30, and backward
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        if(currentTime>=2.45 && currentTime<3.30)//2.31
        BananaPreSets.cargoPickUp();

        //alternative
        //if(currentTime>=3.30 && currentTime<6.35)//6.0
        //BananaDriveStraight.driveStraight(rightTurn);// right turn for left side, left turn for right side
        //if(currentTime>=6.35 && currentTime<8.85)
        //BananaDriveStraight.driveStraight(straightForwardSide);
        //if(currentTime>=8.90 && currentTime<10.70)//9.20
        //BananaClaw.closeClaw(1);
        //if(currentTime>=10.70 && currentTime<10.75)
        //BananaClaw.closeClaw(0);
        //if(currentTime>=10.80 & currentTime<11.80)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=11.85 && currentTime<14.85)
        //BananaDriveStraight.driveStraight(leftTurn);
        //if(currentTime>=14.70 && currentTime<15.0)
        //BananaDriveStraight.driveStraight(straightForwardSide);
       
        


        //



        if(currentTime>=3.30 && currentTime<3.60)//takes 2.10 seconds to aquire a good grip from full open
        BananaDriveStraight.driveStraight(tipToeForward);
        if(currentTime>=3.30 && currentTime<5.40)
        BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        if (currentTime>=5.40 && currentTime<5.45)
        BananaClaw.closeClaw(0);
        if(currentTime>=5.45 && currentTime<6.35)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=6.40 && currentTime<7.30)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=7.35 && currentTime<10.25)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        ////Robot.driveTrain.aimPIDState = true;
        if(currentTime>=10.55 && currentTime<11.05) //giving 0.6 second for claw to open
        BananaClaw.openClaw(1);
        if(currentTime>=11.05 && currentTime<11.10)
        BananaClaw.openClaw(0);
        ////Robot.driveTrain.aimPIDState = false;
        if(currentTime>=11.15 && currentTime<12.65)
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second
        if(currentTime>=12.70 && currentTime<13.0)
        BananaPreSets.lvl2RocketBall();
        if(currentTime>=13.0 && currentTime<13.60)
        BananaPreSets.cargoPickUp();
        if(currentTime>=13.75 && currentTime<14.1) //+0.6 extra to complete the 180 turn
        BananaDriveStraight.driveStraight(hyperRightTurn);
        if(currentTime>=14.1 && currentTime<15.0)
        BananaDriveStraight.driveStraight(straightForwardSide);
        
        

        //if(currentTime>=10.10 && currentTime<13.0)
        //BananaDriveStraight.driveStraight(6);
        //BananaTurn.turnPID(leftFullTurn, 0.0);
        //if(currentTime>=13.05 && currentTime<14)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        
        //if(currentTime>=12.15 && currentTime<12.75) //0.6 seconds to reclose around a cone
        //BananaClaw.closeClaw(1.0);
        //if(currentTime>=12.75 && currentTime<12.80)
        //BananaClaw.closeClaw(0.0);
        //if(currentTime>=12.80 && currentTime<13.80)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=13.80 && currentTime<14.80)
        //BananaTurn.turnPID(leftFullTurn, 0.0); //this method has a wait time of 1 second
        //if(currentTime>=14.80 && currentTime<15.0)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        //ENDS AT 15
        
        

    }

    public static void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        
        //comment out everything and uncomment each line after testing each time
 
        
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=1.0 && currentTime<2.30)//2.30, and backward
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        //if(currentTime>=2.40 && currentTime<3.30)//2.31
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=3.30 && currentTime<5.40)//takes 2.10 seconds to aquire a good grip from full open
        //BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        //if (currentTime>=5.40 && currentTime<5.45)
        //BananaClaw.closeClaw(0);
        //if(currentTime>=5.45 && currentTime<6.35)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=6.40 && currentTime<7.30)
        //BananaPreSets.lvl3RocketBall();
        //if(currentTime>=7.35 && currentTime<10.50)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        //BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        ////Robot.driveTrain.aimPIDState = true;
        //if(currentTime>=10.55 && currentTime<11.05) //giving 0.6 second for claw to open
        //BananaClaw.openClaw(1);
        //if(currentTime>=11.05 && currentTime<11.10)
        //BananaClaw.openClaw(0);
        ////Robot.driveTrain.aimPIDState = false;
        //if(currentTime>=11.15 && currentTime<12.65)
        //BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second
        //if(currentTime>=12.70 && currentTime<13.0)
        //BananaPreSets.lvl2RocketBall();
        //if(currentTime>=13.0 && currentTime<13.60)
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=13.75 && currentTime<15.45) //+0.6 extra to complete the 180 turn
        //BananaDriveStraight.driveStraight(rightTurn);


        
        //if(currentTime>=10.10 && currentTime<13.0)
        //BananaDriveStraight.driveStraight(6);
        //BananaTurn.turnPID(leftFullTurn, 0.0);
        //if(currentTime>=13.05 && currentTime<14)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        
        //if(currentTime>=12.15 && currentTime<12.75) //0.6 seconds to reclose around a cone
        //BananaClaw.closeClaw(1.0);
        //if(currentTime>=12.75 && currentTime<12.80)
        //BananaClaw.closeClaw(0.0);
        //if(currentTime>=12.80 && currentTime<13.80)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=13.80 && currentTime<14.80)
        //BananaTurn.turnPID(leftFullTurn, 0.0); //this method has a wait time of 1 second
        //if(currentTime>=14.80 && currentTime<15.0)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        //ENDS AT 15
        
    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public static void midScorePark() 
    {
        
        if(currentTime>0.0 && currentTime<1.0)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=1.0 && currentTime<2.30)//2.30, and backward
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        //if(currentTime>=2.40 && currentTime<3.30)//2.31
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=3.30 && currentTime<5.40)//takes 2.10 seconds to aquire a good grip from full open
        //BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        //if (currentTime>=5.40 && currentTime<5.45)
        //BananaClaw.closeClaw(0);
        //if(currentTime>=5.45 && currentTime<6.35)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=6.40 && currentTime<7.30)
        //BananaPreSets.lvl3RocketBall();
        //if(currentTime>=7.35 && currentTime<10.50)//10.98, Daniel is sending the video, but I think it went past the goal for 0.5 seconds
        //BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        ////Robot.driveTrain.aimPIDState = true;
        //if(currentTime>=10.55 && currentTime<11.05) //giving 0.6 second for claw to open
        //BananaClaw.openClaw(1);
        //if(currentTime>=11.05 && currentTime<11.10)
        //BananaClaw.openClaw(0);
        ////Robot.driveTrain.aimPIDState = false;
        //if(currentTime>=11.15 && currentTime<12.65)
        //BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second
        //if(currentTime>=12.70 && currentTime<13.0)
        //BananaPreSets.lvl2RocketBall();
        //if(currentTime>=13.0 && currentTime<13.60)
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=13.75 && currentTime<15.45) //+0.6 extra to complete the 180 turn
        //BananaDriveStraight.driveStraight(leftTurn);
        //if(currentTime>=x && currentTime<y)
        //BananaDriveStraight.driveStraight(straightForwardMid);
        //if(currentTime>=7.1 && currentTime<8.1)
        //BananaDriveStraight.driveStraight(tipToeForward);

    } 
}
