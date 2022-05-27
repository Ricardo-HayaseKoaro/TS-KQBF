package problems.kqbf.solvers;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.TreeMap;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import metaheuristics.tabusearch.AbstractTS;
import problems.kqbf.KQBF_Inverse;
import solutions.Solution;


/**
 * Metaheuristic TS (Tabu Search) for obtaining an optimal solution to a QBF
 * (Quadractive Binary Function -- {@link #QuadracticBinaryFunction}).
 * Since by default this TS considers minimization problems, an inverse QBF
 *  function is adopted.
 * 
 * @author ccavellucci, fusberti
 */
public class DIV_TS_KQBF extends AbstractTS<Integer> {
	
	private final Integer fake = new Integer(-1);

	private Map<Integer, Integer> itemFrequency;

	/**
	 * Constructor for the TS_KQBF class. An inverse KQBF objective function is
	 * passed as argument for the superclass constructor.
	 * 
	 * @param tenure
	 *            The Tabu tenure parameter.
	 * @param iterations
	 *            The number of iterations which the TS will be executed.
	 * @param filename
	 *            Name of the file for which the objective function parameters
	 *            should be read.
	 * @throws IOException
	 *             necessary for I/O operations.
	 */
	public DIV_TS_KQBF(Integer tenure, Integer iterations, String filename) throws IOException {
		super(new KQBF_Inverse(filename), tenure, iterations);
		itemFrequency = new TreeMap<>();
	}

	/* (non-Javadoc)
	 * @see metaheuristics.tabusearch.AbstractTS#makeCL()
	 */
	@Override
	public ArrayList<Integer> makeCL() {

		ArrayList<Integer> _CL = new ArrayList<Integer>();
		for (int i = 0; i < ObjFunction.getDomainSize(); i++) {
			Integer cand = new Integer(i);
			_CL.add(cand);
		}

		return _CL;

	}

	/* (non-Javadoc)
	 * @see metaheuristics.tabusearch.AbstractTS#makeRCL()
	 */
	@Override
	public ArrayList<Integer> makeRCL() {

		ArrayList<Integer> _RCL = new ArrayList<Integer>();

		return _RCL;

	}
	
	/* (non-Javadoc)
	 * @see metaheuristics.tabusearch.AbstractTS#makeTL()
	 */
	@Override
	public ArrayDeque<Integer> makeTL() {

		ArrayDeque<Integer> _TS = new ArrayDeque<Integer>(2*tenure);
		for (int i=0; i<2*tenure; i++) {
			_TS.add(fake);
		}

		return _TS;

	}

	/* (non-Javadoc)
	 * @see metaheuristics.tabusearch.AbstractTS#updateCL()
	 */
	@Override
	public void updateCL() {

		ArrayList<Integer> newCL = new ArrayList<Integer>();

		for (Integer cand : CL) {
			if (ObjFunction.getKnapsackWeightByItem(cand) + ObjFunction.getCurrentKnapsackWeight(sol) <= ObjFunction.getKnapsackCapacity()) {
				newCL.add(cand);
			}
		}
		CL = newCL;
	}

	/**
	 * {@inheritDoc}
	 * 
	 * This createEmptySol instantiates an empty solution and it attributes a
	 * zero cost, since it is known that a QBF solution with all variables set
	 * to zero has also zero cost.
	 */
	@Override
	public Solution<Integer> createEmptySol() {
		Solution<Integer> sol = new Solution<Integer>();
		sol.cost = 0.0;
		return sol;
	}

	/**
	 * {@inheritDoc}
	 * 
	 * The local search operator developed for the QBF objective function is
	 * composed by the neighborhood moves Insertion, Removal and 2-Exchange.
	 */
	@Override
	public Solution<Integer> neighborhoodMoveBestImprove() {

		Double minDeltaCost;
		Integer bestCandIn = null, bestCandOut = null;

		minDeltaCost = Double.POSITIVE_INFINITY;
		updateCL();
		// Evaluate insertions
		for (Integer candIn : CL) {
			Double deltaCost = ObjFunction.evaluateInsertionCost(candIn, sol);
			if (!TL.contains(candIn) || sol.cost+deltaCost < bestSol.cost) {
				if (deltaCost < minDeltaCost) {
					minDeltaCost = deltaCost;
					bestCandIn = candIn;
					bestCandOut = null;
				}
			}
		}
		// Evaluate removals
		for (Integer candOut : sol) {
			Double deltaCost = ObjFunction.evaluateRemovalCost(candOut, sol);
			if (!TL.contains(candOut) || sol.cost+deltaCost < bestSol.cost) {
				if (deltaCost < minDeltaCost) {
					minDeltaCost = deltaCost;
					bestCandIn = null;
					bestCandOut = candOut;
				}
			}
		}
		// Evaluate exchanges
		for (Integer candIn : CL) {
			for (Integer candOut : sol) {
				Double deltaCost = ObjFunction.evaluateExchangeCost(candIn, candOut, sol);
				if ((!TL.contains(candIn) && !TL.contains(candOut)) || sol.cost+deltaCost < bestSol.cost) {
					if (deltaCost < minDeltaCost) {
						minDeltaCost = deltaCost;
						bestCandIn = candIn;
						bestCandOut = candOut;
					}
				}
			}
		}
		// Implement the best non-tabu move
		TL.poll();
		if (bestCandOut != null) {
			sol.remove(bestCandOut);
			CL.add(bestCandOut);
			TL.add(bestCandOut);
			itemFrequency.merge(bestCandOut, 1, Integer::sum);
		} else {
			TL.add(fake);
		}
		TL.poll();
		if (bestCandIn != null) {
			sol.add(bestCandIn);
			CL.remove(bestCandIn);
			TL.add(bestCandIn);
			itemFrequency.merge(bestCandIn, 1, Integer::sum);
		} else {
			TL.add(fake);
		}
		ObjFunction.evaluate(sol);
		
		return null;
	}

	public Solution<Integer> neighborhoodMoveFirstImprove() {

		Double minDeltaCost;
		Integer bestCandIn = null, bestCandOut = null;
		boolean foundNewLocalOptimum = false;

		minDeltaCost = Double.POSITIVE_INFINITY;
		updateCL();

		// Evaluate insertions
		if (!foundNewLocalOptimum) {
			for (Integer candIn : CL) {
				Double deltaCost = ObjFunction.evaluateInsertionCost(candIn, sol);
				if (!TL.contains(candIn) || sol.cost+deltaCost < bestSol.cost) {
					if (deltaCost < minDeltaCost) {
						minDeltaCost = deltaCost;
						bestCandIn = candIn;
						bestCandOut = null;
						foundNewLocalOptimum = true;
					}
				}
			}
		}
		
		if (!foundNewLocalOptimum) {
			// Evaluate removals
			for (Integer candOut : sol) {
				Double deltaCost = ObjFunction.evaluateRemovalCost(candOut, sol);
				if (!TL.contains(candOut) || sol.cost+deltaCost < bestSol.cost) {
					if (deltaCost < minDeltaCost) {
						minDeltaCost = deltaCost;
						bestCandIn = null;
						bestCandOut = candOut;
						foundNewLocalOptimum = true;
					}
				}
			}
		}
		
		// Evaluate exchanges
		if (!foundNewLocalOptimum) {
			for (Integer candIn : CL) {
				for (Integer candOut : sol) {
					Double deltaCost = ObjFunction.evaluateExchangeCost(candIn, candOut, sol);
					if ((!TL.contains(candIn) && !TL.contains(candOut)) || sol.cost+deltaCost < bestSol.cost) {
						if (deltaCost < minDeltaCost) {
							minDeltaCost = deltaCost;
							bestCandIn = candIn;
							bestCandOut = candOut;
							foundNewLocalOptimum = true;
						}
					}
				}
			}
		}

		// Implement the best non-tabu move
		TL.poll();
		if (bestCandOut != null) {
			sol.remove(bestCandOut);
			CL.add(bestCandOut);
			TL.add(bestCandOut);
		} else {
			TL.add(fake);
		}
		TL.poll();
		if (bestCandIn != null) {
			sol.add(bestCandIn);
			CL.remove(bestCandIn);
			TL.add(bestCandIn);
			itemFrequency.merge(bestCandIn, 1, Integer::sum);
		} else {
			TL.add(fake);
		}
		ObjFunction.evaluate(sol);
		
		return null;
	}

	/*
	Create solution
	Search neighborhoods keeping track of the frequency a item was included in the solution
	*/
	@Override
	public Solution<Integer> solve() {

		bestSol = createEmptySol();
		constructiveHeuristic();
		TL = makeTL();
		itemFrequency = makeItemFrequency();
		for (int i = 0; i < iterations/2; i++) {
			neighborhoodMoveFirstImprove();
			if (bestSol.cost > sol.cost) {
				bestSol = new Solution<Integer>(sol);
				if (verbose)
					System.out.println("(Iter. " + i + ") BestSol = " + bestSol);
			}
		}

		return restartDiversificationSolution();

	}

	private Map<Integer, Integer> makeItemFrequency() {
		Map<Integer, Integer> _itemFrequency = new HashMap<>();
		for (int i = 0; i < ObjFunction.getDomainSize(); i++) {
			Integer cand = new Integer(i);
			_itemFrequency.put(cand, 0);
		}

		return _itemFrequency;
	}

	private void forceDiversification() {
		// Sort items by frequency
		itemFrequency = itemFrequency.entrySet().stream()
                         .sorted(Entry.comparingByValue())
                         .collect(Collectors.toMap(Entry::getKey, Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
		
		ArrayList<Integer>leastFrequentItems = new ArrayList<>();
		int count = 0;
		for (Integer cand : itemFrequency.keySet()) {
			if (count == itemFrequency.size()/2 ) {
				break;
			}
			leastFrequentItems.add(cand);
		}
		
		Collections.shuffle(leastFrequentItems);		

		// // Random pick an element from solution, exchange for a element
		Random rng = new Random();
		int i = 0;

		Solution<Integer> newSol = new Solution<>(sol);
		for (Integer solItem : sol) {

			Integer cand = leastFrequentItems.get(i);
			while (!CL.contains(cand)) {
				i++;
				if (i >= leastFrequentItems.size()) {
					break;
				}
				cand = leastFrequentItems.get(i);
			}

			if (i >= leastFrequentItems.size()) {
				break;
			}
			if (rng.nextInt(9) > 7) {
				newSol.remove(solItem);
				CL.add(solItem);
				newSol.add(cand);
				CL.remove(cand);
				i++;
			}
		}
		ObjFunction.evaluate(newSol);

		System.out.print(newSol.cost);
		sol = newSol;
	}

	/* 
	Restart search after forcing the least frequent elements to the urrent solution
	*/
	private Solution<Integer> restartDiversificationSolution() {

		TL = makeTL();
		forceDiversification();
		for (int i = 0; i < iterations/2; i++) {
			neighborhoodMoveFirstImprove();
			if (bestSol.cost > sol.cost) {
				bestSol = new Solution<Integer>(sol);
				if (verbose)
					System.out.println("(Iter. " + i + ") BestSol = " + bestSol);
			}
		}
		return bestSol;
	}
	
	/**
	 * A main method used for testing the TS metaheuristic.
	 * 
	 */
	public static void main(String[] args) throws IOException {

		long startTime = System.currentTimeMillis();
		DIV_TS_KQBF tabusearch = new DIV_TS_KQBF(100, 1000, "instances/kqbf/kqbf400");
		Solution<Integer> bestSol = tabusearch.solve();
		System.out.println("maxVal = " + bestSol);
		long endTime   = System.currentTimeMillis();
		long totalTime = endTime - startTime;
		System.out.println("Time = "+(double)totalTime/(double)1000+" seg");

	}

}
