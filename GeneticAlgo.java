package ga;

import java.util.ArrayList;

public abstract class GeneticAlgo {
	
	protected int nChromosomes;
	protected int nGenerations;
	
	protected int currentGeneration = 0;
	
	protected ArrayList<Object> chromosomes = null;
	
	public GeneticAlgo(int nChromosomes, int nGenerations) {
		this.nChromosomes = nChromosomes;
		this.nGenerations = nGenerations;
	}
	
	abstract public double[] calculateFitness();
	
	abstract public void selection();
	
	abstract public void crossover();
	
	abstract public void mutation();
	
	/**
	 * run next generation
	 */
	public void run() {
		this.currentGeneration++;
		this.selection();
		this.crossover();
		this.mutation();
	}
	
	public boolean exitCondition() {
		if (this.currentGeneration<=this.nGenerations) {
			return false;
		}
		return true;
	}
	
}