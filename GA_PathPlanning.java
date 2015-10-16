package ga;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class GA_PathPlanning extends GeneticAlgo {
	public static final int VIA_POINTS_DISTANCE = 40;
	
	public static final int VIA_POINT_UPPER_LIMIT = 300;
	public static final int VIA_POINT_LOWER_LIMIT = -300;
	
	private Robot robot;
	private Point goal;
	private Obstacle[] obstacles;
	
	private int angle;
	private Point deviation;
	
	private int nViaPoints;

	public GA_PathPlanning(int nChromosomes, int nGenerations) {
		super(nChromosomes, nGenerations);
	}
	
	public void initialize(Robot start, Point goal, Obstacle[] obstacles) {
		this.robot = start;
		this.goal = goal;
		this.obstacles = obstacles;
		
		// calculate the no of via points using the VIA_POINTS_DISTANCE
		this.nViaPoints = (int) (Math.ceil((double)this.goal.x / VIA_POINTS_DISTANCE) - 1);
		
		this.chromosomes = new ArrayList<Object>(this.nChromosomes);

		
		transformPoints();
		
		selectInitialChromosomes();
	}
	
	/**
	 * Transform the startPoint, goalPoint and obstaclePoints to the local axis
	 * calculate angle of transformation and the x-deviation to reform the change
	 */
	private void transformPoints() {

		// angle between x-axis and the line between start and goal
//		this.angle = inverse_tan (slope)
		this.angle = (int) Math.toDegrees(Math.atan2(goal.getY()-robot.getY(), goal.getX()-robot.getX()));

		// deviation of start point's x-axis from (0,0)
		this.deviation = this.robot;

		this.robot = new Robot(GA_PathPlanning.Transform(robot, -deviation.x, -deviation.y));
		this.robot = new Robot(Rotate(robot, -angle));
		 
		this.goal = GA_PathPlanning.Transform(goal, -deviation.x, -deviation.y);
		this.goal = GA_PathPlanning.Rotate(goal, -angle);
		
		for(int i=0;i<obstacles.length;i++)
		{
			this.obstacles[i]= new Obstacle(GA_PathPlanning.Transform(obstacles[i], -deviation.x, -deviation.y));
			this.obstacles[i]= new Obstacle(GA_PathPlanning.Rotate(obstacles[i], -angle));
		}
		// make start point as (0,0)
		// transform xy-axis to axis of the straight line between start and goal
		
		
	}
	public static Point Rotate(Point p1 ,int angle)
	{
		int x=(int) ((p1.x)*Math.cos(Math.toRadians(angle))-(p1.y)*Math.sin(Math.toRadians(angle)));
		int y=(int) ((p1.x)*Math.sin(Math.toRadians(angle))+(p1.y)*Math.cos(Math.toRadians(angle)));
		return new Point(x, y);	
	}
	public static Point Transform(Point p,int tx,int ty)
	{
		int x = p.x+tx;
		int y = p.y+ty;
		return new Point(x, y);
	}
	
	public  Point reform(Point p) {
		Point p1=new Point();
		p1=GA_PathPlanning.Rotate(p, angle );
		p1=GA_PathPlanning.Transform(p1,deviation.x,deviation.y);
		
		return p1;
	}
	
	/**
	 * Minimize
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public Point[] path() {
		
		while (!this.exitCondition()) {
			this.run();
		}
		
		double[] chrFitness = calculateFitness();
		ArrayList<Integer> bestChr = (ArrayList<Integer>) this.chromosomes.get(0);
		double maxFitness = chrFitness[0];
		for(int i=1; i<this.nChromosomes; i++) {
			if (maxFitness < chrFitness[i]) {
				maxFitness = chrFitness[i];
				bestChr = (ArrayList<Integer>) this.chromosomes.get(i);
			}
		}
		
		System.out.println(bestChr);
		
		Point[] path = new Point[this.nViaPoints];
		for (int i=0; i<nViaPoints; i++) {
			path[i] = new Point((i+1)*VIA_POINTS_DISTANCE, bestChr.get(i).intValue());
		}
		
		// reform all the via points to the original xy axis
		for (int i=0; i<nViaPoints; i++) {
			path[i] = reform(path[i]);
		}
		

		for(int i=0;i<obstacles.length;i++)
		{
			this.obstacles[i]= new Obstacle(reform(this.obstacles[i]));
		}
		
		/*for (Point point : path) {
			System.out.println(point);
		}*/
		
		return path;
	}
	
	public void selectInitialChromosomes() {
		for (int i=0; i<nChromosomes; i++) {
			ArrayList<Integer> chromosome = new ArrayList<Integer>(this.nViaPoints);
			for (int j=0; j<nViaPoints; j++) {
				int rand = ((int)(Math.random()*(VIA_POINT_UPPER_LIMIT - VIA_POINT_LOWER_LIMIT))) + VIA_POINT_LOWER_LIMIT;
				chromosome.add(new Integer(rand));
			}
			chromosomes.add(chromosome);
		}
		this.printChromosomes("Inital chromosomes");
	}

	@Override
	public double[] calculateFitness() {
		final double ALPHA = 300;
		final double BETA = 0.01;
		double[] result = new double[this.nChromosomes];
		
		for (int i=0; i<this.nChromosomes; i++) {
			@SuppressWarnings("unchecked")
			ArrayList<Integer> chromosome = (ArrayList<Integer>) this.chromosomes.get(i);
			double rateSum = 0;
			double minDisSum = 0;
			for (int j=0; j<this.nViaPoints; j++) {
				Point viaPoint = new Point((j+1)*VIA_POINTS_DISTANCE, chromosome.get(j).intValue());
				double Li = this.goal.distance(viaPoint.x, 0);
				double li = this.goal.distance(viaPoint);
				rateSum += Li / li;
				
				double minDis = 0;
				if (obstacles != null) {
					if (obstacles.length>0) {
						minDis = this.obstacles[0].distance(viaPoint);
					}
					for (int k=1; k<this.obstacles.length; k++) {
						double obsDis = this.obstacles[k].distance(viaPoint);
						if (obsDis < minDis) {
							minDis = obsDis;
						}
					}
				}
				minDisSum += minDis;
			}
			// fitness
			result[i] = (ALPHA*rateSum) + (BETA*minDisSum);
		}
		return result;
	}

	/**
	 * RW Selection
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void selection() {
		ArrayList<Object> matingPool = new ArrayList<Object>();
		double[] chrFitness = calculateFitness();
		double[] cumFitness = new double[this.nChromosomes];
		double totalCumFitness = 0;
		for (int i=0; i<this.nChromosomes; i++) {
			totalCumFitness += chrFitness[i];
			cumFitness[i] = totalCumFitness;
		}
		for (int i=0; i<this.nChromosomes; i++) {
			double rand = Math.random();
			for (int j=0; j<this.nChromosomes; j++) {
				if (rand < cumFitness[j]/totalCumFitness) {
					matingPool.add(new ArrayList<Integer>((ArrayList<Integer>)this.chromosomes.get(j)));
					break;
				}
			}
		}
		this.chromosomes = matingPool;
		this.printChromosomes("After selection");
	}

	/**
	 * Uniform crossover is made between the chromosomes
	 * 	- alternate via-points are swapped
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void crossover() {
		final double CROSSOVER_PROB = 0.5;
		for (int i=0; i<this.nChromosomes-(this.nChromosomes%2); i=i+2) {
			if (Math.random() < CROSSOVER_PROB) {
				ArrayList<Integer> chr1 = (ArrayList<Integer>) this.chromosomes.get(i);
				ArrayList<Integer> chr2 = (ArrayList<Integer>) this.chromosomes.get(i+1);
				// Uniform crossover
				if (!chr1.equals(chr2)) {
					for (int j=0; j<this.nViaPoints; j=j+2) {
						// swap via points between 2 chromosomes
						Integer tmp = chr2.get(j);
						chr2.remove(j);
						chr2.add(j, chr1.get(j));
						chr1.remove(j);
						chr1.add(j, tmp);
					}
				}
			}
		}
		
		this.printChromosomes("After crossover");
	}

	/**
	 * Mutation is done by changing the sign of the via point
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void mutation() {
		final double MUTATION_PROB = 0.1;
		for (int i=0; i<this.nChromosomes; i++) {
			ArrayList<Integer> chr = (ArrayList<Integer>) this.chromosomes.get(i);
			for (int j=0; j<this.nViaPoints; j++) {
				double rand = Math.random();
				if (rand < MUTATION_PROB) {
					// change the sign
					Integer vPoint = chr.get(j);
					chr.remove(j);
					chr.add(j, new Integer(- (vPoint.intValue()%100)));
				}
			}
		}
		this.printChromosomes("After Mutation");
	}
	
	private void printChromosomes(String msg) {
		/*System.out.println(msg);
		for (Object chr : this.chromosomes) {
			System.out.println(chr);
		}
		System.out.println("-------------");*/
	}

}

