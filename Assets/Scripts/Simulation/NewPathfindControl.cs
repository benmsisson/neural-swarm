using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class NewPathfindControl : MonoBehaviour {
	// Convert from world to grid divide by GRID_STEP
	// Convert from grid to world multiply by GRID_STEP
	// The smaller GRID_STEP is, the more granular our map

	// When GRID_STEP is 1, there is no difference between world coordinates and grid coordinates.


	private readonly float GRID_STEP = 1f;
	// Overestimation of sqrt(2). If we are within one GRID_STEP square of the goal we are good to stop
	private readonly float CUTOFF_DIST = 1.5f;
	// The max number of colliders we will check against
	private readonly int NUM_COLLIDERS = 10;
	private readonly float CHECK_DISTANCE = 2f;

	private List<Node> gps;

	private bool[,] grid;
	private ContactFilter2D cf;

	// @Javiar: This is the basic struct for the graph built off the grid.
	private struct GraphPoint {
		public Vector2 gridPos;
		public List<GraphPoint> neighbors;

		public GraphPoint(Vector2 pos) {
			this.gridPos = pos;
			neighbors = new List<GraphPoint>();
		}
	}

	private struct Node {
		public GraphPoint gp;
		public float g;
		public float h;
		public float d;
		public List<Vector2> path;

		public Node(GraphPoint pos, float g, float h) {
			this.gp = pos;
			this.g = g;
			this.h = h;
			this.d = g + h;
			path = new List<Vector2>();
			path.Add(pos.gridPos);
		}

		public Node(GraphPoint pos, float g, float h, Node parent) : this(pos, g, h) {
			path = new List<Vector2>(parent.path);
			path.Add(pos.gridPos);
		}
	}


	public void InitializeGrid(FlockControl.UnityState us) {

		initCF(us);

		CircleCollider2D cd = GetComponent<CircleCollider2D>();
		

		// If GRID_STEP != 1, raycasting falls apart
		Debug.Assert(GRID_STEP == 1);

		Collider2D[] others = new Collider2D[NUM_COLLIDERS];
		grid = new bool[(int)(us.roomWidth / GRID_STEP), (int)(us.roomHeight / GRID_STEP)];


		for (float x = 0; x < us.roomWidth; x += GRID_STEP) {
			for (float y = 0; y < us.roomHeight; y += GRID_STEP) {
				transform.position = new Vector2(x, y);
				int hit = cd.OverlapCollider(cf, others);
				int gx = (int)(x / GRID_STEP);
				int gy = (int)(y / GRID_STEP);
				if (Vector2.SqrMagnitude(new Vector2(x, y) - (Vector2)us.goal.transform.position) < us.goal.transform.localScale.x * us.goal.transform.localScale.x) {
					grid [gx, gy] = true;
					continue;
				}

				grid [gx, gy] = hit == 0;
			}
		}
		// Make a list of nodes, each with their own gridPos
		gps = new List<Node>();
		for (float x = 0; x < us.roomWidth; x += GRID_STEP) {
			for (float y = 0; y < us.roomHeight; y += GRID_STEP) {
				Vector2 point = new Vector2(x, y);
				GraphPoint gp = new GraphPoint(point);
				// Of course g is 0 because we have not travelled anywhere
				Node temp = new Node(gp, 0, Vector2.Distance(point, (Vector2)us.goal.transform.position));
				//if true bellow add node to the list.
				if (hasBlockNeighbor(temp)) {
					gps.Add(temp);
				} 
			}
		}

		// If the node can see the other node, raycast using it 
		foreach (Node p in gps) {
			foreach (Node g in gps) {
				if (p.gp.gridPos == g.gp.gridPos) {
					continue;
				}
				raycastNeighbor(p, g);
			}
		}
		addPoint(us.goal.transform.position);
	}

	private void raycastNeighbor(Node p, Node g) {
		RaycastHit2D[] hs = new RaycastHit2D[1];
		int hit = Physics2D.Raycast(g.gp.gridPos, p.gp.gridPos, cf, hs);
		if (hit == 0) {
			g.gp.neighbors.Add(p.gp);
			p.gp.neighbors.Add(g.gp);
		}
	}

	private Node addPoint(Vector2 gridPos) {
		foreach (Node p in gps) {
			if (p.gp.gridPos == gridPos) {
				// We already made a node for this point, so our work is done
				return p;
			}
		}

		GraphPoint gp = new GraphPoint(gridPos);
		Node newNode = new Node(gp, 0, 0);
		foreach (Node p in gps) {
			raycastNeighbor(newNode, p);			
		}
		gps.Add(newNode);
		return newNode;
	}

	private void initCF(FlockControl.UnityState us) {
		gameObject.transform.localScale = new Vector3(CHECK_DISTANCE * us.maxSize, CHECK_DISTANCE * us.maxSize);
		cf = new ContactFilter2D();
		cf.useTriggers = true;
		cf.layerMask = LayerMask.GetMask("Wall");
		cf.useLayerMask = true;
	}

	public Vector2[] CalculatePath(Vector2 goalPos, BirdControl me) {
		Debug.Log(gps.Count + "," + grid.GetLength(0));

		Vector2 myGridPos = me.transform.position / GRID_STEP;
		Node start = addPoint(myGridPos);


		HashSet<Node> open = new HashSet<Node>();
		open.Add(start);

		HashSet<Vector2> examined = new HashSet<Vector2>();
		examined.Add(myGridPos);

		Node final = new Node();
		bool found = false;

		while (open.Count > 0) {
			float d = Mathf.Infinity;
			Node cur = new Node();
			foreach (Node n in open) {
				if (n.d < d) {
					d = n.d;
					cur = n;
				}
			}

			examined.Add(cur.gp.gridPos);

			if (cur.gp.gridPos == goalPos) {
				final = cur;
				break;
			}

			foreach (GraphPoint p in cur.gp.neighbors) {
				if (examined.Contains(p.gridPos)) {
					continue;
				}
				float ng = Vector2.Distance(cur.gp.gridPos, p.gridPos); 
				float h = Vector2.Distance(p.gridPos, goalPos);
				// TODO: This isn't really the cleanest way to do it. We are making nodes for the graph and for the graph.
				Node n = new Node(p, cur.g + ng, h, cur);
				open.Add(n);
			}
		}

		if (!found) {
			return new Vector2[0];
		}

		Vector2[] positions = getWorldPath(final);

		return positions;
	}

	private Vector2[] getWorldPath(Node n) {
		Vector2[] positions = new Vector2[n.path.Count];
		int i = 0;
		foreach (Vector2 p in n.path) {
			positions [i] = p * GRID_STEP;
			i++;
		}
		return positions;
	}

	private void drawPath(Vector2[] positions, Color c) {
		for (int i = 0; i < positions.Length - 1; i++) {
			Debug.DrawLine(positions [i], positions [i + 1], Color.red);
		}
	}


	private bool hasBlockNeighbor(Node node) {
		for (int xo = -1; xo <= 1; xo++) {
			for (int yo = -1; yo <= 1; yo++) {
				int x = (int)node.gp.gridPos.x + xo;
				int y = (int)node.gp.gridPos.y + yo;
				if (!gridAt(x, y)) {
					return true;
				}
			}
		}
		return false;
	}


	// @Javiar: This is a helper function to look at the grid at a grid x and grid y. It is better than grid[x,y] because it checks the bounds.
	// if it isn't in the bounds, treat it as false (because we cannot go there).
	private bool gridAt(int x, int y) {
		if (x < 0 || x >= grid.GetLength(0)) {
			return false;
		}
		if (y < 0 || y >= grid.GetLength(1)) {
			return false;
		}
		return grid [x, y];
	}

}