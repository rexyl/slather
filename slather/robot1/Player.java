package slather.robot1;

import slather.sim.Cell;
import slather.sim.Point;
import slather.sim.Move;
import slather.sim.Pherome;
import java.util.*;


public class Player implements slather.sim.Player {
    
	private Random gen;
    private double d;
    private int t;
    private int side_length;

    public void init(double d, int t, int side_length) {
	gen = new Random();
	this.d = d;
	this.t = t;
	this.side_length = side_length;
    }

    public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
        Point[] specials = new Point[4];
        specials[0] = new Point(25, 25);
        specials[1] = new Point(75, 25);
        specials[2] = new Point(25, 75);
        specials[3] = new Point(75, 75);
        Point special = null;
    	int special_i = -1;
    	//System.out.println("id: " + player_cell.id + " memory: " + memory);
        // reproduce whenever possible
        if (player_cell.getDiameter() >= 2) {
        	System.out.println("reproduced");
            return new Move(true, (byte)memory, (byte)memory);
        }
        
        boolean just_collided = false;
        for(Cell x : nearby_cells) {
        	if(x.distance(player_cell) < 0.00001 && (memory < 8 || memory > 15)) {
        		just_collided = true;
        	}
        }
        if(just_collided) {
        	System.out.println("collide");
        	special_i = memory&3;
        	special = specials[special_i];
        	Cell sibling = null;
    		for(Cell x : nearby_cells) {
    			if(player_cell.distance(x) <= 0.000001) {
    				sibling = x;
    			}
    		}
    		Point v_player = getClosestDirection(special, player_cell.getPosition());
    		Point v_sibling = getClosestDirection(special, sibling.getPosition());
    		double angle_player = Math.atan2(v_player.y, v_player.x);
    		double angle_sibling = Math.atan2(v_sibling.y, v_sibling.x);
    		if((angle_player - angle_sibling >0 && angle_player - angle_sibling < Math.PI) 
    				|| (angle_player + 2*Math.PI - angle_sibling > 0 && angle_player + 2*Math.PI - angle_sibling < Math.PI)) {
    			memory = (byte) ((2<<2) + (memory&3));
    			return new Move(new Point(0,0), memory);
    		} else {
    			memory = (byte)((3<<2) + (memory&3));
    			return new Move(new Point(0,0), memory);
    		}
        }
        
    	//1 cell, move to center stage
    	if(memory == 0) {
    		double[] dist = new double[4];
    		double min_dist = 200;
    		for(int i=0; i<4; ++i) {
    			dist[i] = getDistance(player_cell.getPosition(), specials[i]);
    			if (dist[i] < min_dist) {
    				min_dist = dist[i];
    				special = specials[i];
    				special_i = i;
    			}
    		}
    		//System.out.println(special_i);
    		if(min_dist > 3)
    			return new Move(getClosestDirection(player_cell.getPosition(), special), (byte)0); 
    		else 
    			return new Move(getClosestDirection(player_cell.getPosition(), special), (byte)(special_i + (1<<2)));
    	//rotate, 1 cell
    	} else if(memory>=4 && memory <=7){
    		special_i = memory&3;
    		special = specials[special_i];
    		int min_radius = 1;// random
    		double ideal_radius = 1/(2*Math.sin(Math.PI/t));
    		if (getDistance(special, player_cell.getPosition()) < Math.max(min_radius, ideal_radius)) {
    			//System.out.println("ideal1: " + ideal_radius + " "+getDistance(special, player_cell.getPosition()));
        		return new Move(nextDirection(player_cell, special, 0, nearby_cells, nearby_pheromes), 
        				(byte)((1<<2) + special_i));
        	} else {
        		double radius = Math.max(min_radius,ideal_radius);
        		//System.out.println("ideal2: " + ideal_radius + " " + getDistance(special, player_cell.getPosition()));
        		
        		return new Move(getNextDirection2(player_cell, 0, special, nearby_cells, nearby_pheromes),
        				(byte)((1<<2) + special_i));
        	}
        //after reproducing
    	} else if(memory >= 8 && memory <= 11) {
    		special_i = memory&3;
    		special = specials[special_i];
    		int min_radius = 1;// random
    		double ideal_radius = 1/(2*Math.sin(Math.PI/t));
    		if (getDistance(special, player_cell.getPosition()) < Math.max(min_radius, ideal_radius)) {
    			//System.out.println("ideal1: " + ideal_radius + " "+getDistance(special, player_cell.getPosition()));
        		return new Move(nextDirection(player_cell, special, 0, nearby_cells, nearby_pheromes), 
        				memory);
        	} else {
        		double radius = Math.max(min_radius,ideal_radius);
        		//System.out.println("ideal2: " + ideal_radius + " " + getDistance(special, player_cell.getPosition()));
        		
        		return new Move(getNextDirection2(player_cell, 0, special, nearby_cells, nearby_pheromes),
        				memory);
        	}
    		
    	} else if(memory >= 12 && memory <= 15) {
    		special_i = memory&3;
    		special = specials[special_i];
    		int min_radius = 1;// random
    		double ideal_radius = 1/(2*Math.sin(Math.PI/t));
    		if (getDistance(special, player_cell.getPosition()) < Math.max(min_radius, ideal_radius)) {
    			//System.out.println("ideal1: " + ideal_radius + " "+getDistance(special, player_cell.getPosition()));
        		return new Move(nextDirection(player_cell, special, 1, nearby_cells, nearby_pheromes), 
        				memory);
        	} else {
        		double radius = Math.max(min_radius,ideal_radius);
        		//System.out.println("ideal2: " + ideal_radius + " " + getDistance(special, player_cell.getPosition()));
        		
        		return new Move(getNextDirection2(player_cell, 1, special, nearby_cells, nearby_pheromes),
        				memory);
        	}
    	} else {
    		System.out.println("memory: " + memory);
    		return null;
    	}
    }

    private double getDistanceDirect(Point first, Point second) {
		double dist_square = (first.x - second.x)*(first.x - second.x) + (first.y - second.y)*(first.y - second.y);
    	double dist = Math.sqrt(dist_square);
    	return dist;
	}

    private double getDistance(Point first, Point second) {
    	double x = second.x;
    	double y = second.y;
    	double dist = 100;
    	for(int area_x = -1; area_x <= 1; area_x ++) {
    		for(int area_y = -1; area_y <= 1; area_y ++) {
    			x = second.x + area_x*100;
    			y = second.y + area_y*100;
    			double d = getDistanceDirect(first, new Point(x,y));
    			if( dist > d) {
    				dist = d;
    			}
    		}
    	}
    	//System.out.println("distance to " + second.x + " " + second.y +"= " + dist);
    	return dist;
    }
    //of first to second
    private Point getClosestDirection(Point first, Point second) {
    	double x = second.x;
    	double y = second.y;
    	double dist = 100;
    	Point best = null;
    	for(int area_x = -1; area_x <= 1; area_x ++) {
    		for(int area_y = -1; area_y <= 1; area_y ++) {
    			x = second.x + area_x*100;
    			y = second.y + area_y*100;
    			double d = getDistanceDirect(first, new Point(x ,y ));
    			if( dist > d) {
    				dist = d;
    				best = new Point(x-first.x,y-first.y);
    			}
    		}
    	}
		
    	return getUnitVector(best);
    }
    
    private Point getUnitVector(Point point) {
    	double x = point.x, y = point.y;
    	double norm = Math.hypot(x, y);
    	x /= norm;
    	y /= norm;
		
    	return new Point(x, y);
    }
    
	private Point nextDirection(Cell player_cell, Point centre, int rad, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {

		double radius = getDistance(player_cell.getPosition(), centre);
		Point outward = (getClosestDirection(centre, player_cell.getPosition()));
		//System.out.println(outward.x + " " + outward.y);
		double angle;
		if(rad==0)
			angle = 3*Math.PI/2;
		else 
			angle = Math.PI/2;
		Point perpendicular = rotate_counter_clockwise(outward, angle);
		return perpendicular;
		
		
    }
	
	private Point getNextDirection2(Cell player_cell, int rad, Point centre, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {

		//System.out.println("hi");
		Point outward = (getClosestDirection(centre, player_cell.getPosition()));
		double rot_angle;
		if(rad==0)
			rot_angle = 3*Math.PI/2;
		else 
			rot_angle = Math.PI/2;
		Point perpendicular = rotate_counter_clockwise(outward, rot_angle);
		
		
        

        double angle = Math.atan2(perpendicular.y, perpendicular.x);
        
        //System.out.format("pheromeps: %f %f%n",closest_to_one_mm.getPosition().x, closest_to_one_mm.getPosition().y);
        //System.out.format("position1: %f %f%n",player_cell.getPosition().x, player_cell.getPosition().y);
       
        double next_angle;
        if(rad==0) next_angle = angle - ((Math.PI)/ (t));
        else next_angle = angle + ((Math.PI)/ (t));
        double dx = Cell.move_dist * Math.cos(next_angle);
        double dy = Cell.move_dist * Math.sin(next_angle);
        return new Point(dx, dy);
	}
	
	//source: https://www.siggraph.org/education/materials/HyperGraph/modeling/mod_tran/2drota.htm
	//for the math
	//angle in radians
	Point rotate_counter_clockwise(Point vector, double angle) {
		double newx, newy,x,y;
		x = vector.x;
		y = vector.y;
		newx = x*Math.cos(angle) - y*Math.sin(angle);
		newy = y*Math.cos(angle) + x*Math.sin(angle);
		return new Point(newx, newy);
	}
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
    private Point extractVectorFromAngle(int arg) {
    double theta = Math.toRadians( 2* (double)arg );
    double dx = Cell.move_dist * Math.cos(theta);
    double dy = Cell.move_dist * Math.sin(theta);
    return new Point(dx, dy);
    }

}
