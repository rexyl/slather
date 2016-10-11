package slather.followAndSquare;
import slather.sim.Cell;
import slather.sim.Point;
import slather.sim.Move;
import slather.sim.Pherome;
import java.util.*;

public class Player implements slather.sim.Player {
    
    private Random gen;

    int t_;
    double d_;
    HashMap<String, Point> fourDirections = new HashMap<String, Point>();
    public void init(double d, int t) {
        gen = new Random();
        t_ = t;
        d_ = d;
        fourDirections.put("right", new Point(Cell.move_dist,0));
        fourDirections.put("left", new Point(-Cell.move_dist,0));
        fourDirections.put("up", new Point(0,Cell.move_dist));
        fourDirections.put("down", new Point(0,-Cell.move_dist));
    }

    public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
        // reproduce whenever possible
        if (player_cell.getDiameter() >= 2) {
            return new Move(true, (byte)1, (byte)-1);
        }
        if (memory >= 0) {   //leader
            for(String k:fourDirections.keySet()){
                Point vector = fourDirections.get(k);
                if (!collides(player_cell, vector, nearby_cells, nearby_pheromes)) {
                    return new Move(vector, memory);
                }
            }
            return new Move(new Point(0,0), memory);

        }
        else{   //follower
            double min_distance = Double.MAX_VALUE;
            Pherome closest_pherome = null;
            for (Pherome p : nearby_pheromes) {
                double distance = player_cell.distance(p);
                if(distance < min_distance) {
                    min_distance = distance;
                    closest_pherome = p;
                }
            }
            if (min_distance <= Cell.move_dist){ //enough to move through
                Point vector = new Point(closest_pherome.getPosition().x - player_cell.getPosition().x, 
                                        closest_pherome.getPosition().y - player_cell.getPosition().y);
                if (!collides(player_cell, vector, nearby_cells, nearby_pheromes))
                    return new Move(vector, memory);
                else
                    return new Move(new Point(0,0), memory);
            }else{
                double angle = Math.atan2(closest_pherome.getPosition().y - player_cell.getPosition().y,
                                 poppp       closest_pherome.getPosition().x - player_cell.getPosition().x);
                double dx = Cell.move_dist * Math.cos(angle);
                double dy = Cell.move_dist * Math.sin(angle);
                Point vector = new Point(dx,dy);
                if (!collides(player_cell, vector, nearby_cells, nearby_pheromes))
                    return new Move(vector, memory);
                else
                    return new Move(new Point(0,0), memory);
            }	
        }
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
