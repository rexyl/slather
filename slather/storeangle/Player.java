package slather.storeangle;
import slather.sim.Cell;
import slather.sim.Point;
import slather.sim.Move;
import slather.sim.Pherome;
import java.util.*;

public class Player implements slather.sim.Player {
    
    private Random gen;

    int t_;
    double d_;

    public void init(double d, int t) {
        gen = new Random();
        t_ = t;
        d_ = d;
    }

    public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
        // reproduce whenever possible
        if (player_cell.getDiameter() >= 2) {
        	double curAngle = (double)2*((int)memory+128);
        	double childAngle = calcPerpendicularAngle(curAngle);
            return new Move(true, memory, mapAngleToRemainingByte(childAngle));
        }
        
        if (memory < 52) { // if angle is 0 to 359, 52 is 180. 52 to 127 (75)remaining
            double curAngle = (double)2*((int)memory+128);
            double nextAngle = calcNextAngle(curAngle,t_);
            Point vector = extractVectorFromAngle(nextAngle);
            if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
                return new Move(vector, calByteFromAngle(nextAngle));
            }
            else{
                //TODO: if collide with friend? enemy? pherome?
                return new Move(new Point(0,0), memory);
            }
            // Point vector = extractVectorFromAngle( (double)2*(int)memory);
            // nextDirection(player_cell,memory)
            // check for collisions
            // if (!collides( player_cell, vector, nearby_cells, nearby_pheromes))
            //     return new Move(vector, memory);
        }
        else{
        	double angle = mapRemainingByteToAngle(memory);
        	Point vector = extractVectorFromAngle(angle);
        	if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
        		return new Move(vector, calByteFromAngle(angle));
            }	
        }

        // if no previous direction specified or if there was a collision, try random directions to go in until one doesn't collide
        for (int i=0; i<4; i++) {
            int angle = gen.nextInt(360);
            Point vector = extractVectorFromAngle((double)angle);
            if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) 
                return new Move(vector, calByteFromAngle(angle));
        }

        // if all tries fail, just chill in place
        return new Move(new Point(0,0), (byte)0);
    }

    //private Point nextDirection(Cell player_cell,byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
        // double curAngle = (double)2*(int)memory;
        // double nextAngle = calcNextAngle(curAngle,t_);
        // Point vector = extractVectorFromAngle(nextAngle);
        // if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
        //     return vector;
        // }
        // else{
        //     return new Point(0,0);
        // }
        // Pherome closest_to_one_mm = null;
        // double min_distance = Double.MAX_VALUE;

        // if (nearby_pheromes.isEmpty()) {
        //     for (int i=0; i<4; i++) {
        //         int arg = gen.nextInt(180)+1;
        //         Point vector = extractVectorFromAngle((double)2*arg);
        //         if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
        //             return vector;
        //         }
        //     }
        // } else {
        //     for (Pherome p : nearby_pheromes) {

        //         double distance = Math.abs(1 - player_cell.distance(p));
        //         if(distance < min_distance) {
        //             min_distance = distance;
        //             closest_to_one_mm = p;
        //         }
        //     }

        //     Point cur_direction = new Point(player_cell.getPosition().x - closest_to_one_mm.getPosition().x, 
        //                                     player_cell.getPosition().y - closest_to_one_mm.getPosition().y);

        //     double angle = Math.atan2(cur_direction.y, cur_direction.x);

        //     double next_angle = angle + ((2*Math.PI)/ (t_-1));
        //     double dx = Cell.move_dist * Math.cos(next_angle);
        //     double dy = Cell.move_dist * Math.sin(next_angle);
        //     Point vector = new Point(dx,dy);
        //     if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
        //         return vector;
        //     }
        // }

        // return new Point(0,0);
    //}

    // check if moving player_cell by vector collides with any nearby cell or hostile pherome
    private boolean collides(Cell player_cell, Point vector, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
    Iterator<Cell> cell_it = nearby_cells.iterator();
    Point destination = player_cell.getPosition().move(vector);
    while (cell_it.hasNext()) {
        Cell other = cell_it.next();
        if ( destination.distance(other.getPosition()) < 0.5*player_cell.getDiameter() + 0.5*other.getDiameter() + 0.00011) 
        return true;
    }
    Iterator<Pherome> pherome_it = nearby_pheromes.iterator();
    while (pherome_it.hasNext()) {
        Pherome other = pherome_it.next();
        if (other.player != player_cell.player && destination.distance(other.getPosition()) < 0.5*player_cell.getDiameter() + 0.0001) 
        return true;
    }
    return false;
    }

    // convert an angle (in 2-deg increments) to a vector with magnitude Cell.move_dist (max allowed movement distance)
    private Point extractVectorFromAngle(double arg) {
        double theta = Math.toRadians( arg );
        double dx = Cell.move_dist * Math.cos(theta);
        double dy = Cell.move_dist * Math.sin(theta);
        return new Point(dx, dy);
    }

    private double calcNextAngle(double angle, int t) {
        double nextAngle = angle + (double)360/(t+1);
        if(nextAngle >= 360)
            return nextAngle - 360;
        else
            return nextAngle;
    }

    private byte calByteFromAngle(double angle){
        return (byte)((int)Math.ceil(angle/2)-128);
    }
    private byte calByteFromAngle(int angle){
        return (byte)((int)Math.ceil(angle/2.0)-128);
    }

    private double mapRemainingByteToAngle(byte memory){
        return ((int)memory-52)/75.0*360.0;
    }

    private byte mapAngleToRemainingByte(double angle){
        return (byte)((angle/360)*75+52);
    }
    
    private double calcPerpendicularAngle(double angle){
    	return angle-90;
    }
}
