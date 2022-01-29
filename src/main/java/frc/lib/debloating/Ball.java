package frc.lib.debloating;

import edu.wpi.first.wpilibj.I2C.Port;

public class Ball {
    private Position position = Position.UNKNOWN;
    private Color color = Color.UNKNOWN;
    private Direction direction = Direction.UNKNOWN;
    private SensorSnapshot prevTraversalSnapshot = null;
    private SensorSnapshot currentTraversalSnapshot = null;
    private SensorSnapshot prevTriggerSnapshot = null;
    private SensorSnapshot currentTriggerSnapshot = null;

    public boolean isAbove(Position otherBall){
        int thisPositionLevel = getPosition().ordinal();
        int otherPositionLevel = otherBall.ordinal();
        return thisPositionLevel >= otherPositionLevel;
    }

    public Position getPosition(){
        return position;
    }

    public void setMotion(Direction direction){
        this.direction = direction;
    }

    boolean topBallResolved = false;
    boolean topBall = false;

    public void update(ColorSensor traversalSensor, ColorSensor triggerSensor, Ball otherBall){
        prevTraversalSnapshot = currentTraversalSnapshot;
        currentTraversalSnapshot = new SensorSnapshot(traversalSensor);

        prevTriggerSnapshot = currentTriggerSnapshot;
        currentTriggerSnapshot = new SensorSnapshot(triggerSensor);

        //skips first and second run without enough data
        if(prevTraversalSnapshot == null || prevTriggerSnapshot == null) return;

        boolean ballHasMovedIntoTraversal = prevTraversalSnapshot.ballPresent() ? 
            false : currentTraversalSnapshot.ballPresent();

        boolean ballHasMovedOutOfTraversal = prevTraversalSnapshot.ballPresent() ? 
            !currentTraversalSnapshot.ballPresent() : false;

        boolean ballHasMovedIntoTrigger = prevTriggerSnapshot.ballPresent() ? 
            false : currentTriggerSnapshot.ballPresent();

        boolean ballHasMovedOutOfTrigger = prevTriggerSnapshot.ballPresent() ? 
            !currentTriggerSnapshot.ballPresent() : false;

            
        boolean positionResolved = false;
        boolean colorResolved = color != Color.UNKNOWN;

        if(otherBall.position == Position.UNKNOWN || position == Position.UNKNOWN){
            topBallResolved = false;
        }
        if(!topBallResolved){
            topBall = isAbove(otherBall.getPosition());
            topBallResolved = true;
        }

        if((position == Position.UNKNOWN || 
            position == Position.BELOW_TRAVERSAL_SENSOR) &&
            direction == Direction.UPWARDS){
            if (ballHasMovedIntoTraversal){
                position = Position.IN_TRAVERSAL;
                positionResolved = true;
                if(!colorResolved) {
                    color = currentTraversalSnapshot.ballColor();
                    colorResolved = true;
                }
            }
        }
        if(position == Position.IN_TRAVERSAL && !positionResolved){
            if(ballHasMovedOutOfTraversal){
                if(direction == Direction.UPWARDS){
                    position = Position.ABOVE_TRAVERSAL_SENSOR;
                    positionResolved = true;
                } else if (direction == Direction.DOWNWARDS){
                    position = Position.BELOW_TRAVERSAL_SENSOR;
                    positionResolved = true;
                }
            }
        }
        if(position == Position.ABOVE_TRAVERSAL_SENSOR && !positionResolved) {
            if(ballHasMovedIntoTrigger) {
                if(direction == Direction.UPWARDS) {
                    position = Position.IN_TRIGGER;
                    positionResolved = true;
                    if(!colorResolved) {
                        color = currentTriggerSnapshot.ballColor();
                        colorResolved = true;
                    }
                    
                }
            } else if(ballHasMovedIntoTraversal) {
                if(direction == Direction.DOWNWARDS) {
                    position = Position.IN_TRAVERSAL;
                    positionResolved = true;
                    if(!colorResolved) {
                        color = currentTraversalSnapshot.ballColor();
                        colorResolved = true;
                    }
                }
            }
        }
        if(position == Position.IN_TRIGGER && !positionResolved) {
            if(ballHasMovedOutOfTrigger){
                if(direction == Direction.UPWARDS){
                    position = Position.UNKNOWN;
                    positionResolved = true;
                } else if (direction == Direction.DOWNWARDS){
                    position = Position.ABOVE_TRAVERSAL_SENSOR;
                    positionResolved = true;
                }
            }
        }
        if(!topBall && topBallResolved && isAbove(otherBall.getPosition()) && positionResolved){
            position = Position.values()[position.ordinal() - 1];
        }
    }
    
    public enum Position{
        UNKNOWN,
        BELOW_TRAVERSAL_SENSOR,
        IN_TRAVERSAL,
        ABOVE_TRAVERSAL_SENSOR,
        IN_TRIGGER
    }
    public enum Color{
        UNKNOWN,
        RED,
        BLUE
    }
    public enum Direction{
        UNKNOWN,
        UPWARDS,
        DOWNWARDS
    }
    public static class SensorSnapshot{
        private final boolean ballPresent;
        private Color ballColor = Color.UNKNOWN;

        public SensorSnapshot(ColorSensor sensor){
            ballPresent = sensor.objPresent();
            if(ballPresent){
                ballColor = sensor.isRedNotBlue() ? Color.RED : Color.BLUE;
            }
        }

        public boolean ballPresent(){
            return ballPresent;
        }

        public Color ballColor(){
            return ballColor;
        }
    }

    //test main
    public static void main(String[] args){
        Ball ball1 = new Ball();
        Ball ball2 = new Ball();
        //do stuff
    }
}
