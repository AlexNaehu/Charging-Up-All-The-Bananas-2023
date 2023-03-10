package frc.robot.Autonomous;


import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Robot.BananaPreSets;
import frc.robot.Robot.Robot;

public class BananaAutonPaths {





    





    private final static double leftHalfTurn = 90.0; //"left turn" means the front right corner whips around CCW
    private final static double leftFullTurn = 180;
    private final static double rightHalfTurn = -90.0; //"right turn" means the front left corner whips around CW
    private final static double rightFullTurn = -180;

    

    private final static int straightForwardSide = 0;
    private final static int straightForwardMid = 1;
    private final static int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final static int straightBackwardSide = 3;
    private final static int straightBackwardMid = 4;
    private final static int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring

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
 
        //if(currentTime>0.05 && currentTime<1.1)
        //BananaPreSets.cargoPickUp();
        if(currentTime>=0.1 && currentTime<0.5)
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        //if(currentTime>=1.5 && currentTime<2.1)
        //BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        //if (currentTime>=2.1 && currentTime<2.15)
        //BananaClaw.closeClaw(0);
        //if(currentTime>=2.15 && currentTime<3.15)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=3.15 && currentTime<3.30)
        //BananaClaw.openClaw(1); //Loosen grip to do the spinney thing with the preloaded cone
        //if(currentTime>=3.30 && currentTime<3.35)
        //BananaClaw.openClaw(0);
        //if(currentTime>=3.40 && currentTime<3.60)
        //BananaClaw.closeClaw(1);
        //if(currentTime>=3.60 && currentTime<3.65)
        //BananaClaw.closeClaw(0);
        //if(currentTime>=3.5 && currentTime<3.55)
        //BananaPreSets.lvl3RocketBall();
        //if(currentTime>=2.35 && currentTime<2.40)
        //BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        ////Robot.driveTrain.aimPIDState = true;
        //if(currentTime>=4.35 && currentTime<4.95) //giving 0.6 second for claw to open
        //BananaClaw.openClaw(1);
        //if(currentTime>=4.95 && currentTime<5.0)
        //BananaClaw.openClaw(0);
        //Robot.driveTrain.aimPIDState = false;
        //if(currentTime>=4.0 && currentTime<4.05)
        //BananaDriveStraight.driveStraight(straightBackwardSide); //this method has a wait time of 1 second
        //if(currentTime>=6.05 && currentTime<7.05)
        //BananaPreSets.cargoPickUp();
        //if(currentTime>=6.05 && currentTime<6.10)
        //BananaTurn.turnPID(leftFullTurn, 0.0);
        //if(currentTime>=7.05 && currentTime<7.1)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        //if(currentTime>=9.05 && currentTime<10.05)
        //BananaClaw.closeClaw(1.0);
        //if(currentTime>=10.05 && currentTime<10.1)
        //BananaClaw.closeClaw(0.0);
        //if(currentTime>=10.1 && currentTime<11.1)
        //BananaPreSets.lvl1RocketBall();
        //if(currentTime>=10.1 && currentTime<10.15)
        //BananaTurn.turnPID(leftFullTurn, 0.0); //this method has a wait time of 1 second
        //if(currentTime>=11.1 && currentTime<11.15)
        //BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        //if(currentTime>=12.1 && currentTime<12.15)
        //BananaDriveStraight.driveStraight(straightForwardSide); //EXTRA Driving Forward Because We Have Spare Time
        //ENDS AT 14.1
        
        

    }

    public static void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        
        //comment out everything and uncomment each line after testing each time
 
        if(currentTime>0.05 && currentTime<1.1)
        BananaPreSets.cargoPickUp();
        if(currentTime>=0.1 && currentTime<0.15)
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        if(currentTime>=1.5 && currentTime<2.1)
        BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        if (currentTime>=2.1 && currentTime<2.15)
        BananaClaw.closeClaw(0);
        if(currentTime>=2.15 && currentTime<3.15)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=3.15 && currentTime<3.30)
        BananaClaw.openClaw(1); //Loosen grip to do the spinney thing with the preloaded cone
        if(currentTime>=3.30 && currentTime<3.35)
        BananaClaw.openClaw(0);
        if(currentTime>=3.40 && currentTime<3.60)
        BananaClaw.closeClaw(1);
        if(currentTime>=3.60 && currentTime<3.65)
        BananaClaw.closeClaw(0);
        if(currentTime>=3.5 && currentTime<3.55)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=2.35 && currentTime<2.40)
        BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        //Robot.driveTrain.aimPIDState = true;
        if(currentTime>=4.35 && currentTime<4.95) //giving 0.6 second for claw to open
        BananaClaw.openClaw(1);
        if(currentTime>=4.95 && currentTime<5.0)
        BananaClaw.openClaw(0);
        //Robot.driveTrain.aimPIDState = false;
        if(currentTime>=4.0 && currentTime<4.05)
        BananaDriveStraight.driveStraight(straightBackwardSide); //this method has a wait time of 1 second
        if(currentTime>=6.05 && currentTime<7.05)
        BananaPreSets.cargoPickUp();
        if(currentTime>=6.05 && currentTime<6.10)
        BananaTurn.turnPID(rightFullTurn, 0.0);
        if(currentTime>=7.05 && currentTime<7.1)
        BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        if(currentTime>=9.05 && currentTime<10.05)
        BananaClaw.closeClaw(1.0);
        if(currentTime>=10.05 && currentTime<10.1)
        BananaClaw.closeClaw(0.0);
        if(currentTime>=10.1 && currentTime<11.1)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=10.1 && currentTime<10.15)
        BananaTurn.turnPID(rightFullTurn, 0.0); //this method has a wait time of 1 second
        if(currentTime>=11.1 && currentTime<11.15)
        BananaDriveStraight.driveStraight(straightForwardSide); //this method has a wait time of 1 second
        if(currentTime>=12.1 && currentTime<12.15)
        BananaDriveStraight.driveStraight(straightForwardSide); //EXTRA Driving Forward Because We Have Spare Time
        //ENDS AT 14.1

    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public static void midScorePark() 
    {
        
        if(currentTime>0.05 && currentTime<1.1)
        BananaPreSets.cargoPickUp();
        if(currentTime>=0.1 && currentTime<0.15)
        BananaDriveStraight.driveStraight(tipToeBackward); //this method has a wait time of 1 second, operation time of 1 second
        if(currentTime>=1.1 && currentTime<2.1)
        BananaClaw.closeClaw(1); //in order to do the flippy, we need to grab the cone from the top side to allow the center of mass distribution to be on the bottom side, rotating the cone to face bottom down
        if (currentTime>=2.1 && currentTime<2.15)
        BananaClaw.closeClaw(0);
        if(currentTime>=2.15 && currentTime<3.15)
        BananaPreSets.lvl1RocketBall();
        if(currentTime>=3.15 && currentTime<3.30)
        BananaClaw.openClaw(1); //Loosen grip to do the spinney thing with the preloaded cone
        if(currentTime>=3.30 && currentTime<3.35)
        BananaClaw.openClaw(0);
        if(currentTime>=3.5 && currentTime<3.55)
        BananaPreSets.lvl3RocketBall();
        if(currentTime>=2.35 && currentTime<2.40)
        BananaDriveStraight.driveStraight(tipToeForward); //this method has a wait time of 1 second, operation time of 1 second
        //Robot.driveTrain.aimPIDState = true;
        if(currentTime>=4.35 && currentTime<4.95) //giving 0.6 second for claw to open
        BananaClaw.openClaw(1);
        if(currentTime>=4.95 && currentTime<5.0)
        BananaClaw.openClaw(0);
        //Robot.driveTrain.aimPIDState = false;
        if(currentTime>=4 && currentTime<4.05)
        BananaDriveStraight.driveStraight(tipToeBackward);
        if(currentTime>=4 && currentTime<5)
        BananaClaw.openClaw(1.0);
        if(currentTime>=5.0 && currentTime<5.05)
        BananaClaw.openClaw(0.0);
        if(currentTime>=5.1 && currentTime<6.1)
        BananaPreSets.neutralPivotAngle();
        if(currentTime>=5.1 && currentTime<5.15)
        BananaTurn.turnPID(leftFullTurn, 0.0);
        if(currentTime>=6.1 && currentTime<7.1)
        BananaDriveStraight.driveStraight(straightForwardMid);
        if(currentTime>=7.1 && currentTime<8.1)
        BananaDriveStraight.driveStraight(tipToeForward);

    } 
}
