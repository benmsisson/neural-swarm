﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;

// TODO: Not switching maps at all
// TODO: Maybe don't use mutation chance at all and just have a small mutation
public class ForceGenetic : ForceDNA {
	// Use a high mutation rate because we are not flipping bits but modifying floats
	private static readonly float MUTATION_CHANCE = .25f;
	// How many std deviations we travel in one mutation
	private static readonly float MUTATION_RATE = .5f;


	private static readonly float CONST_MUT = twoItemStDev(CONST_MIN, CONST_MAX)*MUTATION_RATE;

	private static readonly float EXP_MUT = twoItemStDev(EXP_MIN, EXP_MAX)*MUTATION_RATE;

	private static readonly float STEP_MUT = twoItemStDev(STEP_MIN, STEP_MAX)*MUTATION_RATE;

	private static readonly float CARRY_MUT = twoItemStDev(CARRY_MIN, CARRY_MAX)*MUTATION_RATE;

	private static readonly float DIST_MUT = twoItemStDev(DIST_MIN, DIST_MAX)*MUTATION_RATE;

	private static readonly float VIEW_MUT = twoItemStDev(VIEW_MIN, VIEW_MAX)*MUTATION_RATE;


	private static readonly int NUM_SPECIES = 30;

	// If the average score is less than SCORE_CUTOFF*the max average score for SCORE_STRIKES generations, we switch maps
	private static readonly float SCORE_CUTOFF = .95f;
	private static readonly float SCORE_STRIKES = 3;

	private Genome[] genomes;
	private float[] scores;
	private int current;

	private bool readFromFile = false;
	private string uuid = "09acb057154d4aae8457c2761f15121c";
	private int generation = 39;

	private int strikes = 0 ;
	private float maxAverage;

	private FlockControl.RandomDelegate randomizePositions;


	public ForceGenetic(int numBirds, FlockControl.RandomDelegate rp) {
		randomizePositions = rp;
		current = -1;
		if (!readFromFile) {
			initRandom();
		} else {
			initFromFile();
		}
	}

	private void initRandom() {
		genomes = new Genome[NUM_SPECIES];
		scores = new float[NUM_SPECIES];
		for (int i = 0; i < NUM_SPECIES; i++) {
			genomes [i] = new Genome();
		}
		uuid = System.Guid.NewGuid().ToString("N");
		generation = 1;
	}

	private void initFromFile() {
		genomes = FileIO.ReadFromFile(uuid,generation);
		if (genomes == null) {
			Debug.LogError("Could not find file. Using random initialization instead");
			initRandom();
			return;
		}
		scores = new float[genomes.Length];
	}

	// calculate the (population) standard deviation of two items
	private static float twoItemStDev(float num1, float num2) {
		
		double average = ((double)num1 + (double)num2)/2;
		double difference1 = (num1 - average);
		double difference2 = (num2 - average);
		double squared1 = difference1 * difference1;
		double squared2 = difference2 * difference2;
		double squaredAverage = (squared1 + squared2) / 2;
		return (float) System.Math.Sqrt(squaredAverage);
	}

	public override Genome Next() {
		current++;
		Debug.Log(current + "/" + genomes.Length);
		if (current >= genomes.Length) {
			current = 0;
			FileIO.WriteToFile(uuid,generation,genomes,scores);
			evolve();
		}
		return genomes [current];
	}

	private void evolve() {
		generation++;

		Genome[] newGenomes = new Genome[genomes.Length];

		for (int i = 0; i < genomes.Length; i++) {
			int p1 = selectParent(scores);
			int p2 = selectParent(scores);
			Genome g1 = genomes [p1];
			Genome g2 = genomes [p2];
			newGenomes [i] = Crossover(g1, g2);
		}

		genomes = newGenomes;

		float ave = scores.Sum()/scores.Length;
		if (ave>maxAverage) {
			maxAverage = ave;
			strikes = 0;
		} else if (ave<maxAverage*SCORE_CUTOFF){
			strikes++;
		}

		if (strikes>=SCORE_STRIKES) {
			randomizePositions();
			maxAverage = 0;
			strikes = 0;
			Debug.Log("Randomizing positions!");
		}
	}

	private int selectParent(float[] adjusted) {
		float sum = adjusted.Sum();
		float s = Random.Range(0, sum);
		int parent = -1;
		while (s > 0) {
			parent++;
			s -= adjusted [parent];
		}
		return parent;
	}

	public override void Score(float score) {
		scores [current] = score;
	}

	public Genome Crossover(Genome g1, Genome g2) {
		Chromosome ali = Crossover(g1.Align, g2.Align);
		Chromosome coh = Crossover(g1.Cohesion, g2.Cohesion);
		Chromosome rep = Crossover(g1.Repulse, g2.Repulse);
		Chromosome obs = Crossover(g1.Obstacle, g2.Obstacle);
		Chromosome bou = Crossover(g1.Boundary, g2.Boundary);
		Chromosome rew = Crossover(g1.Reward, g2.Reward);
		PathChrom pf = Crossover(g1.Pathfind, g2.Pathfind);
		return new Genome(ali, coh, rep, obs, bou, rew, pf);
	}

	public Chromosome Crossover(Chromosome p1, Chromosome p2) {
		float cm = (Random.value < MUTATION_CHANCE) ? Random.Range(-CONST_MUT, CONST_MUT) : 0;
		float c = (Random.value > .5f ? p1.Constant : p2.Constant) + cm;

		float em = (Random.value < MUTATION_CHANCE) ? Random.Range(-EXP_MUT, EXP_MUT) : 0;
		float e = (Random.value > .5f ? p1.Exponent : p2.Exponent) + em;


		float dm = (Random.value < MUTATION_CHANCE) ? Random.Range(-DIST_MUT, DIST_MUT) : 0;
		float d = (Random.value > .5f ? p1.Distance : p2.Distance) + dm;

		return new Chromosome(c, e, d);
	}

	public PathChrom Crossover(PathChrom p1, PathChrom p2) {
		float sm = (Random.value < MUTATION_CHANCE) ? Random.Range(-STEP_MUT, STEP_MUT) : 0;
		float s = (Random.value > .5f ? p1.Steps : p2.Steps) + sm;

		float cm = (Random.value < MUTATION_CHANCE) ? Random.Range(-CARRY_MUT, CARRY_MUT) : 0;
		float c = (Random.value > .5f ? p1.Carryover : p2.Carryover) + cm;

		float vm = (Random.value < MUTATION_CHANCE) ? Random.Range(-VIEW_MUT, VIEW_MUT) : 0;
		float v = (Random.value > .5f ? p1.View : p2.View) + vm;

		Chromosome cr = Crossover(p1.Chrom, p2.Chrom);
		return new PathChrom(cr, s, c, v);
	}
}