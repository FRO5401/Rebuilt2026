package frc.robot.Utils;

public class FuelTracker {

    private static FuelTracker instance;
    private static int ballCounter = 0;

    private FuelTracker(){}

    public static FuelTracker getInstance(){
        if(instance == null) instance = new FuelTracker();
        return instance;
    }

    public static void addBall(){
        ballCounter++;
    }

    public static void addBalls(int balls){
        ballCounter += balls;
    }

    public static void removeBall(){
        ballCounter--;
    }

    public static void removeBalls(int balls){
        ballCounter -= balls;
        if (ballCounter < 0) resetBallCount();
    }

    public static void resetBallCount(){
        ballCounter = 0;
    }

    public static void setBallCount(int count){
        ballCounter = count;
    }
}
