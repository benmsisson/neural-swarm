﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlockControl : MonoBehaviour {
	public GameObject goalPrefab;
	public GameObject birdPrefab;
	public GameObject wallPrefab;
	public GameObject background;

	private GameObject goal;
	private BirdControl[] birdControls;
	private int reachedGoal;

	private StatsControl statsControl;

	private GameObject[] walls;

	private float startTime = 0;

	private readonly int NUM_BIRDS = 50;
	private readonly float ROOM_WIDTH = 50;
	private readonly float ROOM_HEIGHT = 40;

	private readonly float MIN_SIZE = .75f;
	private readonly float MAX_SIZE = 1.3f;

	private readonly float MIN_SPEED = 4f;
	private readonly float MAX_SPEED = 6f;

	private readonly int NUM_WALLS = 5;
	private readonly float WALL_MAX_WIDTH = 10f;
	private readonly float WALL_MIN_WIDTH = 2f;
	// Walls are constrained to have fixed area, so width = area/height
	private readonly float WALL_MAX_AREA = 12f;
	private readonly float WALL_MIN_AREA = 8f;

	private readonly float MAX_TIME = 25f;


	[System.Serializable]
	struct WorldState {
		public BirdControl.Bird[] birds;
		public Vector2 goalPosition;
	}

	public void Start () {
		// Set the background based on room settings
		background.transform.position = new Vector3(ROOM_WIDTH/2,ROOM_HEIGHT/2,5);
		background.transform.localScale = new Vector3(ROOM_WIDTH+5,ROOM_HEIGHT+5,1);
		background.GetComponent<Renderer>().material.color = Color.black;

		goal = Instantiate<GameObject>(goalPrefab);
		statsControl = FindObjectOfType<StatsControl>();

		// Generate birds
		birdControls = new BirdControl[NUM_BIRDS];
		for (int i = 0; i < NUM_BIRDS; i++) {
			BirdControl bird = Instantiate<GameObject>(birdPrefab).GetComponent<BirdControl>();
			birdControls[i] = bird;
		}

		walls = new GameObject[NUM_WALLS];
		for (int i = 0; i < NUM_WALLS; i++) {
			walls[i] = Instantiate<GameObject>(wallPrefab);
			}

		resetBirds();

	}
	
	public string Serialize() {
		BirdControl.Bird[] birds = new BirdControl.Bird[NUM_BIRDS];
		for (int i = 0; i < NUM_BIRDS; i++) {
			birds[i] = birdControls[i].ToStruct();
		}
		WorldState ws = new WorldState();
		ws.birds = birds;
		ws.goalPosition = (Vector2)goal.transform.position;
		return JsonUtility.ToJson(ws);
	}

	public void IncrementGoal() {
		reachedGoal++;
		if (reachedGoal==NUM_BIRDS){
			statsControl.PrintStats();
			resetBirds();
		}
	}

	private void resetBirds() {
		goal.transform.position = randomPosition();

		reachedGoal = 0;
		for (int i = 0; i < NUM_BIRDS; i++) {
			BirdControl bird = birdControls[i];
			bird.transform.position = randomPosition();
			float size = Random.Range(MIN_SIZE,MAX_SIZE);
			float speed = Random.Range(MIN_SPEED,MAX_SPEED);
			bird.Setup(size,speed,i);
			bird.GetComponent<Renderer>().material.color = new Color(Random.Range(.5f,1f),Random.Range(.5f,1f),Random.Range(.5f,1f));
		}

		for (int i = 0; i < NUM_WALLS; i++) {
			float width = Random.Range(WALL_MIN_WIDTH,WALL_MAX_WIDTH);
			float area = Random.Range(WALL_MIN_AREA,WALL_MAX_AREA);
			walls[i].transform.localScale = new Vector3(width,area/width,1f);
			walls[i].transform.position = randomPosition();
			walls[i].transform.rotation = Quaternion.Euler(0,0,Random.Range(0f,360f));
		}

		statsControl.Setup(NUM_BIRDS,MAX_TIME);
		startTime = Time.time;
	}

	public void Deserialize(string rawCommand) {
		// Expected format is a Python list of lists of two numbers ie [[1,2],[3,4]]
		// Removed the outermost brackets
		rawCommand = rawCommand.Substring(1,rawCommand.Length-2);
		// If we know for sure we had more than 1 bird, we could split by '],[' but instead we must split by '['
		string[] rawSplits = rawCommand.Split(new char[]{'['});

		for (int i = 0; i < rawSplits.Length; i++) {
			// The first string will always be "", so the bird we are on is i-1
			if (rawSplits[i] == "") {
				continue;
			}
			// Splitting by ']' and getting the first element in that split gives us 1,2
			// So split that by ',' to get the raw numbers
			string[] xy = rawSplits[i].Split(new char[]{']'})[0].Split(new char[]{','});
			Vector2 accel = new Vector2(float.Parse(xy[0]),float.Parse(xy[1]));
			birdControls[i-1].SetAcceleration(accel);
		}
	}


	public Rect GetWorldBound() {
		return new Rect(0,0,ROOM_WIDTH,ROOM_HEIGHT);
	}

	private Vector3 randomPosition() {
		return new Vector3(Random.Range(0,ROOM_WIDTH),Random.Range(0,ROOM_HEIGHT),0);
	}

	private void Update() {
		if (Time.time-startTime>MAX_TIME) {
			statsControl.PrintStats();
			resetBirds();
		}
	}
}
