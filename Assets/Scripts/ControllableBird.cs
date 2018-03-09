using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// this class controls the "birds" in Unity. 
// This script is a component of the Bird prefab

public class ControllableBird : MonoBehaviour {
	private Vector3 lastPos;


	private Vector2 velocity;
	private Vector2 accel;

	private FlockControl flockControl;
	private StatsControl statsControl;
	private float size;
	private float speed;
	private float mass;
	private int number = -1;
	private bool moving = false;

	[System.Serializable]
	public struct Bird {
		public Vector2 position;
		public Vector2 velocity;
		public float size;
		public float speed;
		public float mass;
	}

	public void Setup(float size, float speed, int number) {
		this.flockControl = FindObjectOfType<FlockControl>();
		this.statsControl = FindObjectOfType<StatsControl>();
		this.velocity = Vector2.zero;
		this.size = size;
		this.transform.localScale*=size;
		this.speed = speed;
		this.mass = size*size;
		this.number = number;
		this.lastPos = transform.position;
		moving = true;
		gameObject.GetComponent<Collider2D>().enabled = true;
	}


	public void SetAcceleration(Vector2 accel) {
		if (accel.magnitude > speed*1.001f) {
//			Debug.Log("Setting acceleration too high:" + accel.magnitude + "," + speed);
			accel = Vector2.ClampMagnitude(accel,speed);
		}
		this.accel = accel;
	}

	// Update is called once per frame
	public void Update () {
        
		Vector3 mouse = Input.mousePosition;
        mouse.z = 10f; // Set this to be the distance you want the object to be placed in front of the camera.
        transform.position = Camera.main.ScreenToWorldPoint(mouse);
        
        // manually handle out of bounds, but doesn't work (NullPointerException)
		/*Rect worldBound = flockControl.GetWorldBound();
        if (mouse.x < flockControl.GetWorldBound().xMin) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(.01f,mouse.y));
		}
        
		if (mouse.x > worldBound.xMax) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(worldBound.xMax-.01f,mouse.y)) ;
		}

		if (mouse.y < worldBound.yMin) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(mouse.x,.01f));
		}

		if (mouse.y > worldBound.yMax) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(mouse.x,worldBound.yMax-.01f));
		}
        */


        // commented out because velocity is not calculated for the bird in this situation
        //velocity += accel/mass*Time.deltaTime;
		//velocity = Vector2.ClampMagnitude(velocity,speed);
		//transform.position += (Vector3)velocity*Time.deltaTime;
		//updateRotation();
		
	}

	public void OnTriggerEnter2D(Collider2D collider) {
		if (collider.gameObject.tag=="Bird") {
			ControllableBird other = collider.GetComponent<ControllableBird>();
			if (other.number < number || number == -1) {
				return;
			}
			handleBirdCollision(other);
		}
		if (collider.gameObject.tag=="Goal") {
			moving = false;
			gameObject.GetComponent<Collider2D>().enabled=false;
			statsControl.Complete(number);
			flockControl.IncrementGoal();
		}
		if (collider.gameObject.tag=="Wall") {
			handleWallCollision(collider);
		}
	}

	public void OnTriggerStay2D(Collider2D collider) {
		if (collider.gameObject.tag=="Bird") {
			ColliderDistance2D dist = gameObject.GetComponent<Collider2D>().Distance(collider);
			transform.position += (Vector3) dist.normal*dist.distance/2;
			collider.gameObject.transform.position -= (Vector3) dist.normal*dist.distance/2;
		}
		if (collider.gameObject.tag=="Wall") {
			ColliderDistance2D dist = gameObject.GetComponent<Collider2D>().Distance(collider);
			transform.position += (Vector3) dist.normal*dist.distance;
		}
	}

	public Bird ToStruct() {
		Bird b = new Bird();
		b.mass=mass;
		b.position=transform.position;
		b.size=size;
		b.speed=speed;
		b.velocity=velocity;
		return b;
	}

	private void updateRotation() {
		float rotation = Mathf.Atan2(velocity.y,velocity.x);
		transform.rotation = Quaternion.Euler(new Vector3(0,0,rotation*Mathf.Rad2Deg-90));
	}

	private void handleOutOfBounds(Rect worldBound,Vector3 lastPos, Vector3 mouse) {
		if (worldBound.Contains(transform.position)) {
			return;
		}
			
		if (transform.position.x < worldBound.xMin) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(.01f,mouse.y));
		}
		if (transform.position.x > worldBound.xMax) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(worldBound.xMax-.01f,mouse.y)) ;
		}

		if (transform.position.y < worldBound.yMin) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(mouse.x,.01f));
		}

		if (transform.position.y > worldBound.yMax) {
			transform.position = Camera.main.ScreenToWorldPoint(new Vector3(mouse.x,worldBound.yMax-.01f));
		}

	}
		
	private void handleBirdCollision(ControllableBird other) {
		velocity = getResultantVelocity(transform.position,other.transform.position,mass,other.mass,velocity,other.velocity);
		updateRotation();

		other.velocity = getResultantVelocity(other.transform.position,transform.position,other.mass,mass,other.velocity,velocity);
		other.updateRotation();
		statsControl.AddBirdCollision();

	}


	private static Vector2 getResultantVelocity(Vector2 position1, Vector2 position2, float mass1, float mass2, Vector2 velocity1, Vector2 velocity2) {
		Vector2 posDifference = (Vector2) (position1 - position2);
		float relativeMass;
		if (mass2 == Mathf.Infinity) {
			relativeMass = 2f;
		} else {
			relativeMass = (2*mass2)/(mass1+mass2);
		}
		float vPosDot = Vector2.Dot(velocity1-velocity2,posDifference);
		float posDistance = Mathf.Pow(posDifference.magnitude,2);
		Vector2 myNewVelocity = velocity1 - relativeMass * vPosDot/posDistance * posDifference;
		return myNewVelocity;
	}

	private void handleWallCollision(Collider2D other) {
		ColliderDistance2D dist = gameObject.GetComponent<Collider2D>().Distance(other);
		transform.position += (Vector3) dist.normal*dist.distance;
		velocity = getResultantVelocity(transform.position,dist.pointB,mass,Mathf.Infinity,velocity,Vector2.zero);
		statsControl.AddWallCollision();
	}
}
