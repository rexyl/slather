package slather.g5;

import slather.sim.Cell;
import slather.sim.Point;
import slather.sim.Move;
import slather.sim.Pherome;
import slather.sim.GridObject;
import java.util.*;


public class Player implements slather.sim.Player {
    //arbitrary right now
    private static final double THRESHOLD_DISTANCE = 3;

	private Random gen;
    public AggresivePlayer aggresivePlayer;
    int t_;
    double d_;

    public void init(double d, int t, int side_length) {
        aggresivePlayer = new AggresivePlayer();
        gen = new Random();
        t_ = t;
        d_ = d;
    }

    private Point pathOfLeastResistance(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
    	
    	class GridObjectAnglePair <GridObjectAnglePair>{
            GridObject gridObject;
            Point angle;
            public GridObjectAnglePair(GridObject obj, Point ang) {
                angle = ang;
                gridObject = obj;
            }
        }
        
        class GridObjectAnglePairComparator implements Comparator<GridObjectAnglePair> {
            @Override
            public int compare(GridObjectAnglePair a, GridObjectAnglePair b) {
                double angle1 = Math.atan2(a.angle.y, a.angle.x);
                double angle2 = Math.atan2(b.angle.y, b.angle.x);
                if(angle1==angle2) return 0;
                return angle1<angle2?-1:1;
            }   
        }
        
        List<GridObjectAnglePair> nearby_list = new ArrayList<GridObjectAnglePair>(); 
        for(GridObject cell : nearby_cells) {
            nearby_list.add
            	(new GridObjectAnglePair
            			(cell,
            			getClosestDirection(player_cell.getPosition(), cell.getPosition())));
        }
        for(GridObject pherome : nearby_pheromes) {
            if(pherome.player != player_cell.player) {
                nearby_list.add
                (new GridObjectAnglePair
                		(pherome,
                		getClosestDirection(player_cell.getPosition(), pherome.getPosition())));
            }
        }
        nearby_list.sort(new GridObjectAnglePairComparator());
        if(nearby_list.size()>1) {
            double widest = -1;
            int widest_index = -1;
            for(int i = 0; i < nearby_list.size(); ++i) {
            	Point p0 = nearby_list.get(i-1<0?nearby_list.size()-1:i-1).gridObject.getPosition();
            	Point p1 = nearby_list.get(i).gridObject.getPosition();
                double angle = 
                		angleBetweenVectors(getClosestDirection(player_cell.getPosition(), p0),
                				getClosestDirection(player_cell.getPosition(), p1));
                //System.out.println("angle: " + angle);
                if( widest < angle ) {
                    widest = angle;
                    widest_index = i;
                }
            }
            //System.out.println();
            Point p1 = nearby_list.get(widest_index).angle;
            Point p2 = nearby_list.get(widest_index-1<0?nearby_list.size()-1:widest_index-1).angle;
            
            p2 = rotate_counter_clockwise(p2, widest/2);
            
            
            //return new Move(p3, memory);
            return p2;
        } else if(nearby_list.size() == 1) {
            return new Point(-nearby_list.get(0).angle.x,-nearby_list.get(0).angle.y);
        }
        return new Point(0,0);
    }
    
    /*
     * from p1 to p2 counter clockwise, 0 to 2PI
     */
    private double angleBetweenVectors(Point p1, Point p2) {
    	double x1 = p1.x, x2 = p2.x, y1 = p1.y, y2 = p2.y;
    	double dot = x1*x2 + y1*y2;    
    	double det = x1*y2 - y1*x2;  
    	double angle = Math.atan2(det, dot);
    	if(angle < 0) angle += 2*Math.PI;
    	return angle;
    }
    private Point getLargestTraversableDistance(
    		Point direction,
    		Cell player_cell,
    		Set<Cell> nearby_cells,
    		Set<Pherome> nearby_pheromes) {
    	
    	direction = getUnitVector(direction);
    	double small = 0;
    	double large = 1;
    	while(large - small > 0.001) {
    		double mid = (small + large) / 2;
    		Point vector = new Point(direction.x * mid, direction.y * mid);
    		boolean can_move = !collides(player_cell, vector, nearby_cells, nearby_pheromes);
    		if(can_move) small = mid;
    		else large = mid;
    	}
    	Point nextPoint = new Point(direction.x * small, direction.y * small);
    	
    	return new Point(direction.x * small, direction.y * small);
    }
    private Point pathBetweenTangents(
    		Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {

    	class GridObjectAnglePair{
            GridObject gridObject;
            Point angle;
            public GridObjectAnglePair(GridObject obj, Point ang) {
                angle = ang;
                gridObject = obj;
            }
        }
        
    	
        class GridObjectAnglePairComparator implements Comparator<GridObjectAnglePair> {
            @Override
            public int compare(GridObjectAnglePair a, GridObjectAnglePair b) {
                double angle1 = Math.atan2(a.angle.y, a.angle.x);
                double angle2 = Math.atan2(b.angle.y, b.angle.x);
                if(angle1==angle2) return 0;
                return angle1<angle2?-1:1;
            }   
        }
        
        List<GridObjectAnglePair> nearby_list = new ArrayList<GridObjectAnglePair>(); 
        for(GridObject cell : nearby_cells) {
            nearby_list.add
            	(new GridObjectAnglePair
            			(cell,
            			getClosestDirection(player_cell.getPosition(), cell.getPosition())));
        }
        for(GridObject pherome : nearby_pheromes) {
            if(pherome.player != player_cell.player) {
                nearby_list.add
                (new GridObjectAnglePair
                		(pherome, 
                		getClosestDirection(player_cell.getPosition(), pherome.getPosition())));
            }
        }
        nearby_list.sort(new GridObjectAnglePairComparator());
        if(nearby_list.size()>1) {
        	
        	//start with biggest cell, or 0 if no cells
        	int biggest_i = 0;
        	double big_diam = 0;
        	int ind = 0;
        	for(GridObjectAnglePair g : nearby_list) {
        		if(g.gridObject instanceof Cell) {
        			if(((Cell)g.gridObject).getDiameter() > big_diam) {
        				big_diam = ((Cell)g.gridObject).getDiameter();
        				biggest_i = ind;
        			}
        		}
        		++ind;
        	}
        	
        	
            double widest = -1;
            Point widest_vector = null;
            Point prev_tangent = 
            		(Point) getTangentDirections(player_cell, nearby_list.get(biggest_i).gridObject)
            		.get(1);
            int prev_i = biggest_i;
            int sz = nearby_list.size();
            for(int i = biggest_i + 1; i < nearby_list.size() + biggest_i + 1; ++i) {
            	int k = i%sz;
            	Point p0 = nearby_list.get(prev_i).gridObject.getPosition();
            	Point p1 = nearby_list.get(k).gridObject.getPosition();
            	
            	Point current_tangent1 = 
            			(Point) getTangentDirections(player_cell, nearby_list.get(k).gridObject)
            			.get(0);
            	Point current_tangent2 = 
            			(Point) getTangentDirections(player_cell, nearby_list.get(k).gridObject)
            			.get(1);
            	double angle = angleBetweenVectors(prev_tangent, current_tangent1);
                
                double center_angle = 
                		angleBetweenVectors(getClosestDirection(player_cell.getPosition(), p0),
                				getClosestDirection(player_cell.getPosition(), p1));
                //if overlap occurs
                if(center_angle < Math.PI && angle > Math.PI) {
                	double angle_temp = angleBetweenVectors(current_tangent2, prev_tangent);
                	if(angle_temp < Math.PI) {
                		prev_i = k;
                		prev_tangent = current_tangent2;
                	}
                //no overlap
                } else {
                	if(widest < angle) {
                		widest = angle;
                		widest_vector = prev_tangent;
                	}
                	prev_tangent = current_tangent2;
                	prev_i = k;
                }
            }
            if(widest_vector==null) {
            	//This should also return empty...
            	return pathOfLeastResistance(player_cell, nearby_cells, nearby_pheromes);
            }
            Point p2 = rotate_counter_clockwise(widest_vector, widest/2);
            if(collides(player_cell, p2, nearby_cells, nearby_pheromes)) {
            	
            	return pathOfLeastResistance(player_cell, nearby_cells, nearby_pheromes);
            }
            
            //return new Move(p3, memory);
            return p2;
        } else if(nearby_list.size() == 1) {
            return new Point(-nearby_list.get(0).angle.x,-nearby_list.get(0).angle.y);
        }
        return new Point(0,0);
    }
    
    
    /*
     * Angle in Radian
     */
    Point rotate_counter_clockwise(Point vector, double angle) {
		double newx, newy,x,y;
		x = vector.x;
		y = vector.y;
		newx = x*Math.cos(angle) - y*Math.sin(angle);
		newy = y*Math.cos(angle) + x*Math.sin(angle);
		return new Point(newx, newy);
	}
    
    public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
        //restrict to d_restrict mm sight
    	//System.out.println(nearby_cells.size());
    	
        double d_restrict = 2.0;
        d_restrict = Math.min(d_restrict,d_);
        Set<Cell> nearby_cells_restricted = new HashSet<Cell>();
        Set<Pherome> nearby_pheromes_restricted = new HashSet<Pherome>();
        int enemy_cells = 0;
        for (Cell near_cell :nearby_cells ) {
            if (player_cell.distance(near_cell) <= d_restrict ) {
                nearby_cells_restricted.add(near_cell);
                if (near_cell.player != player_cell.player)
                    enemy_cells++;
            }
        }
        int enemy_pheromes = 0;
        for (Pherome near_pherome :nearby_pheromes ) {
            if (player_cell.distance(near_pherome) <= d_restrict ) {
                nearby_pheromes_restricted.add(near_pherome);
                if (near_pherome.player != player_cell.player)
                    enemy_pheromes++;
            } 
        }

        System.out.println(nearby_cells_restricted.size());

        // reproduce whenever possible
        if (player_cell.getDiameter() >= 2) {
            return new Move(true, (byte)0, (byte)0);
        }

        // if (enemy_pheromes + enemy_cells > 14)
        //     System.out.println(enemy_pheromes + enemy_cells);

        Point nextPath = pathBetweenTangents(player_cell, nearby_cells_restricted, nearby_pheromes_restricted);

        if(nextPath.x != 0 && nextPath.y != 0) {
            if(!collides(player_cell, nextPath, nearby_cells_restricted, nearby_pheromes_restricted)) {
                return new Move(nextPath, (byte)(int)((Math.toDegrees(Math.atan2(nextPath.y, nextPath.x))/2)));
            } else {
            	Point vector = getLargestTraversableDistance(
    					nextPath, player_cell, nearby_cells_restricted, nearby_pheromes_restricted);
            	//System.out.println("newlate2 norm: " + vector.norm());
            	if(vector.norm() < 0.75 && vector.norm() > 0.25 
            			&& !collides(player_cell, vector, nearby_cells_restricted, nearby_pheromes_restricted)) {
            		//System.out.println("newlate2 Well done, strategy.");
            		return new Move(
            			vector, (byte)(int)((Math.toDegrees(Math.atan2(nextPath.y, nextPath.x))/2)));
            	}
            }
        } else {
            // continue moving in the same direction as before
            Point vector = extractVectorFromAngle( (int)memory);
            // check for collisions
            if (!collides( player_cell, vector, nearby_cells_restricted, nearby_pheromes_restricted))
            return new Move(vector, memory);
        }
        

        // Generate a random new direction to travel
        for (int i=0; i<4; i++) {
            int arg = gen.nextInt(180)+1;
            Point vector = extractVectorFromAngle(arg);
            if (!collides(player_cell, vector, nearby_cells_restricted, nearby_pheromes_restricted)) 
            return new Move(vector, (byte) arg);
        }

        // if all tries fail, just chill in place
        return new Move(new Point(0,0), (byte)0);
    }
    
    
    /*
     * Returns in direction player_cell --> other
     * Second one is rotated more counter clockwise than first
     */
    private List<Point> getTangentDirections(Cell player_cell, GridObject other) {
    	List<Point> out = new ArrayList<Point>();
    	double r1 = player_cell.getDiameter()/2;
    	double r2 = 0;
    	if(other instanceof Cell) {
    		r2 = ((Cell)other).getDiameter()/2;
    	}
    	
    	double d = getDistance(player_cell.getPosition(), other.getPosition());
    	double theta2 = Math.asin((r1+r2)/d);
    	Point center_to_center = new Point(other.getPosition().x-player_cell.getPosition().x,
    			other.getPosition().y-player_cell.getPosition().y);
    	center_to_center = getUnitVector(center_to_center);
    	out.add(rotate_counter_clockwise(center_to_center, -theta2));
    	out.add(rotate_counter_clockwise(center_to_center, theta2));
    	
    	
    	return out;
    }
    
    
    // check if moving player_cell by vector collides with any nearby cell or hostile pherome
    private boolean collides(Cell player_cell, Point vector, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
    Iterator<Cell> cell_it = nearby_cells.iterator();
    Point destination = player_cell.getPosition().move(vector);
    while (cell_it.hasNext()) {
        Cell other = cell_it.next();
        if ( destination.distance(other.getPosition()) 
        		< 0.011*player_cell.getDiameter() 
        		+ 0.5*player_cell.getDiameter() 
        		+ 0.5*other.getDiameter() 
        		+ 0.00011) 
        return true;
    }
    Iterator<Pherome> pherome_it = nearby_pheromes.iterator();
    while (pherome_it.hasNext()) {
        Pherome other = pherome_it.next();
        if (other.player != player_cell.player 
        		&& destination.distance(other.getPosition()) 
        		< 0.011*player_cell.getDiameter() + 0.5*player_cell.getDiameter() + 0.0001) 
        return true;
    }
    return false;
    }

    // convert an angle (in 2-deg increments) to a vector with magnitude
    // Cell.move_dist (max allowed movement distance)
    private Point extractVectorFromAngle(int arg) {
    double theta = Math.toRadians( 2* (double)arg );
    double dx = Cell.move_dist * Math.cos(theta);
    double dy = Cell.move_dist * Math.sin(theta);
    return new Point(dx, dy);
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
    	//System.out.println(first.x +" " + first.y +" " + second.x +" "+ second.y);
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
		//if(best == null) System.out.println("best null");
    	return getUnitVector(best);
    }
    
    private Point getUnitVector(Point point) {
    	double x = point.x, y = point.y;
    	double norm = Math.hypot(x, y);
    	x /= norm;
    	y /= norm;
		
    	return new Point(x, y);
    }
    
}
