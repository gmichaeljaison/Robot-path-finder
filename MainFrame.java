package ga;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Point;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.WindowConstants;

public class MainFrame extends JFrame {
	private static final long serialVersionUID = 832931427680133162L;
	private static final int WINDOW_WIDTH = 1440;
	private static final int WINDOW_HEIGHT = 860;
	private static final Point ORIGIN = new Point(WINDOW_WIDTH/4, WINDOW_HEIGHT/2);
	
	protected JPanel drawPanel = null;
	
	private Robot robot = null;
	private Point goalPnt = null;
	private Obstacle[] obstacles = null;
	
	private GA_PathPlanning pathPlanner = null;
	private Point[] pathPnts = null;

	public MainFrame() {
		super("ROBOT Path Planning");
		this.setPreferredSize(new Dimension(WINDOW_WIDTH, WINDOW_HEIGHT));
		this.setBackground(Color.black);
		this.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		
		drawPanel = new JPanel();
		this.add(drawPanel);
		
		this.pack();
		
		pathPlanner = new GA_PathPlanning(50, 500);
		
		robot = new Robot(new Point(0, 0));
		goalPnt = new Point(500, 0);
		
		Point[] obstaclePnts = new Point[2];
		obstaclePnts[0] = new Point(50, 150);
		obstaclePnts[1] = new Point(360, -150);
		
		obstacles = new Obstacle[2];
		obstacles[0] = new Obstacle(obstaclePnts[0]);
		obstacles[1] = new Obstacle(obstaclePnts[1]);
		
		pathPlanner.initialize(robot, goalPnt, obstacles);
		pathPnts = pathPlanner.path();
		for (int i = 0; i < pathPnts.length; i++) {
			pathPnts[i] = getOriginRelPoint(pathPnts[i]);
		}
	}
	
	public void paint(Graphics g) {
		Point relStartPnt = getOriginRelPoint(robot);
		Point relGoalPnt = getOriginRelPoint(goalPnt);
		
		g.setColor(Color.black);
		g.fillRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
		
		// Goal
		g.setColor(Color.yellow);
		g.fillOval(relGoalPnt.x-50/2, relGoalPnt.y-50/2, 50, 50);
		
		robot.drawRobot(g);
		
		for (int i=0; i<obstacles.length; i++) {
			obstacles[i].drawObstacle(g);
		}
		
		// path
		g.setColor(Color.white);
		g.drawLine(relStartPnt.x, relStartPnt.y, pathPnts[0].x, pathPnts[0].y);
		for (int i=0; i<pathPnts.length-1; i++) {
			g.drawLine(pathPnts[i].x, pathPnts[i].y, pathPnts[i+1].x, pathPnts[i+1].y);
		}
		g.drawLine(pathPnts[pathPnts.length-1].x, pathPnts[pathPnts.length-1].y, relGoalPnt.x, relGoalPnt.y);
	}
	
	public void repaint() {
		this.paint(getGraphics());
	}
	
	public static Point getOriginRelPoint(Point p) {
		return new Point(ORIGIN.x+p.x, ORIGIN.y-p.y);
	}
	
}

class Robot extends Point {
	private static final long serialVersionUID = 1L;
	private static final int RADIUS = 50;
	private static final Color COLOR = Color.green;
	
	public Robot(Point p) {
		super(p);
	}
	
	public void drawRobot(Graphics g) {
		Color tmpColor = g.getColor();
		g.setColor(COLOR);
		Point tmp = MainFrame.getOriginRelPoint(this);
		g.fillOval(tmp.x-RADIUS/2, tmp.y-RADIUS/2, RADIUS, RADIUS);
		g.setColor(tmpColor);
	}
}

class Obstacle extends Point {
	private static final long serialVersionUID = 1L;
	private static final int RADIUS = 50;
	private static final Color COLOR = Color.gray;
	
	public Obstacle(Point p) {
		super(p);
	}
	
	public void drawObstacle(Graphics g) {
		Color tmpColor = g.getColor();
		g.setColor(COLOR);
		Point tmp = MainFrame.getOriginRelPoint(this);
		g.fillOval(tmp.x-RADIUS/2, tmp.y-RADIUS/2, RADIUS, RADIUS);
		g.setColor(tmpColor);
	}
	
	public double distance(Point p) {
		return super.distance(p)-RADIUS;
		/*if (super.distance(p) > RADIUS) {
			return super.distance(p)-RADIUS;
		}
		return 0;*/
	}
	
}