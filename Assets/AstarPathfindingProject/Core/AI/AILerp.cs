using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Pathfinding {
	using Pathfinding.Util;

	[RequireComponent(typeof(Seeker))]
	[AddComponentMenu("Pathfinding/AI/AILerp (2D,3D)")]
	
	public class AILerp : VersionedMonoBehaviour, IAstarAI {
		
		public float repathRate = 0.5F;

		
		public bool canSearch = true;

	
		public bool canMove = true;

		
		public float speed = 3;

		
		[UnityEngine.Serialization.FormerlySerializedAs("rotationIn2D")]
		public OrientationMode orientation = OrientationMode.ZAxisForward;

		
		[System.Obsolete("Use orientation instead")]
		public bool rotationIn2D {
			get { return orientation == OrientationMode.YAxisForward; }
			set { orientation = value ? OrientationMode.YAxisForward : OrientationMode.ZAxisForward; }
		}

		
		public bool enableRotation = true;

		public float rotationSpeed = 10;

		
		public bool interpolatePathSwitches = true;

		
		public float switchPathInterpolationSpeed = 5;

		public bool reachedEndOfPath { get; private set; }

		public bool reachedDestination {
			get {
				if (!reachedEndOfPath) return false;
				// Note: distanceToSteeringTarget is the distance to the end of the path when approachingPathEndpoint is true
				var dir = destination - interpolator.endPoint;
				// Ignore either the y or z coordinate depending on if  using 2D mode or not
				if (orientation == OrientationMode.YAxisForward) dir.z = 0;
				else dir.y = 0;

				// Check against using a very small margin
				if (remainingDistance + dir.magnitude >= 0.05f) return false;

				return true;
			}
		}

		public Vector3 destination { get; set; }

		
		[System.NonSerialized]
		public bool updatePosition = true;

		
		[System.NonSerialized]
		public bool updateRotation = true;

		
		[System.Obsolete("Use the destination property or the AIDestinationSetter component instead")]
		public Transform target {
			get {
				var setter = GetComponent<AIDestinationSetter>();
				return setter != null ? setter.target : null;
			}
			set {
				targetCompatibility = null;
				var setter = GetComponent<AIDestinationSetter>();
				if (setter == null) setter = gameObject.AddComponent<AIDestinationSetter>();
				setter.target = value;
				destination = value != null ? value.position : new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
			}
		}

		public Vector3 position { get { return updatePosition ? tr.position : simulatedPosition; } }

		public Quaternion rotation { get { return updateRotation ? tr.rotation : simulatedRotation; } }

		#region IAstarAI implementation

		/// <summary>\copydoc Pathfinding::IAstarAI::Move</summary>
		void IAstarAI.Move (Vector3 deltaPosition) {
			// This script does not know the concept of being away from the path that it is following
			// so this call will be ignored (as is also mentioned in the documentation).
		}

		/// <summary>\copydoc Pathfinding::IAstarAI::radius</summary>
		float IAstarAI.radius { get { return 0; } set {} }

		/// <summary>\copydoc Pathfinding::IAstarAI::height</summary>
		float IAstarAI.height { get { return 0; } set {} }

		/// <summary>\copydoc Pathfinding::IAstarAI::maxSpeed</summary>
		float IAstarAI.maxSpeed { get { return speed; } set { speed = value; } }

		/// <summary>\copydoc Pathfinding::IAstarAI::canSearch</summary>
		bool IAstarAI.canSearch { get { return canSearch; } set { canSearch = value; } }

		/// <summary>\copydoc Pathfinding::IAstarAI::canMove</summary>
		bool IAstarAI.canMove { get { return canMove; } set { canMove = value; } }

		Vector3 IAstarAI.velocity {
			get {
				return Time.deltaTime > 0.00001f ? (previousPosition1 - previousPosition2) / Time.deltaTime : Vector3.zero;
			}
		}

		Vector3 IAstarAI.desiredVelocity {
			get {
				// The AILerp script sets the position every frame. It does not take into account physics
				// or other things. So the velocity should always be the same as the desired velocity.
				return (this as IAstarAI).velocity;
			}
		}

		/// <summary>\copydoc Pathfinding::IAstarAI::steeringTarget</summary>
		Vector3 IAstarAI.steeringTarget {
			get {
				// AILerp doesn't use steering at all, so we will just return a point ahead of the agent in the direction it is moving.
				return interpolator.valid ? interpolator.position + interpolator.tangent : simulatedPosition;
			}
		}
		#endregion

		public float remainingDistance {
			get {
				return Mathf.Max(interpolator.remainingDistance, 0);
			}
			set {
				interpolator.remainingDistance = Mathf.Max(value, 0);
			}
		}

		public bool hasPath {
			get {
				return interpolator.valid;
			}
		}

		public bool pathPending {
			get {
				return !canSearchAgain;
			}
		}

		public bool isStopped { get; set; }

		public System.Action onSearchPath { get; set; }

		protected Seeker seeker;

		protected Transform tr;

		protected float lastRepath = -9999;

		protected ABPath path;

		protected bool canSearchAgain = true;

		
		protected Vector3 previousMovementOrigin;
		protected Vector3 previousMovementDirection;

		
		protected float pathSwitchInterpolationTime = 0;

		protected PathInterpolator interpolator = new PathInterpolator();


		
		bool startHasRun = false;

		Vector3 previousPosition1, previousPosition2, simulatedPosition;
		Quaternion simulatedRotation;

		[UnityEngine.Serialization.FormerlySerializedAs("target")][SerializeField][HideInInspector]
		Transform targetCompatibility;

		protected AILerp () {
			
			destination = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
		}

		
		protected override void Awake () {
			base.Awake();
			tr = transform;

			seeker = GetComponent<Seeker>();

			seeker.startEndModifier.adjustStartPoint = () => simulatedPosition;
		}

		
		protected virtual void Start () {
			startHasRun = true;
			Init();
		}

		
		protected virtual void OnEnable () {
			seeker.pathCallback += OnPathComplete;
			Init();
		}

		void Init () {
			if (startHasRun) {
				Teleport(position, false);
				lastRepath = float.NegativeInfinity;
				if (shouldRecalculatePath) SearchPath();
			}
		}

		public void OnDisable () {
			if (seeker != null) seeker.CancelCurrentPathRequest();
			canSearchAgain = true;

			if (path != null) path.Release(this);
			path = null;
			interpolator.SetPath(null);

			seeker.pathCallback -= OnPathComplete;
		}

		public void Teleport (Vector3 position, bool clearPath = true) {
			if (clearPath) interpolator.SetPath(null);
			simulatedPosition = previousPosition1 = previousPosition2 = position;
			if (updatePosition) tr.position = position;
			reachedEndOfPath = false;
			if (clearPath) SearchPath();
		}

		protected virtual bool shouldRecalculatePath {
			get {
				return Time.time - lastRepath >= repathRate && canSearchAgain && canSearch && !float.IsPositiveInfinity(destination.x);
			}
		}

		
		[System.Obsolete("Use SearchPath instead")]
		public virtual void ForceSearchPath () {
			SearchPath();
		}

		public virtual void SearchPath () {
			if (float.IsPositiveInfinity(destination.x)) return;
			if (onSearchPath != null) onSearchPath();

			lastRepath = Time.time;

			// This is where the path should start to search from
			var currentPosition = GetFeetPosition();

			// If we are following a path, start searching from the node we will
			// reach next this can prevent odd turns right at the start of the path
			/*if (interpolator.valid) {
			    var prevDist = interpolator.distance;
			    // Move to the end of the current segment
			    interpolator.MoveToSegment(interpolator.segmentIndex, 1);
			    currentPosition = interpolator.position;
			    // Move back to the original position
			    interpolator.distance = prevDist;
			}*/

			canSearchAgain = false;

			// Alternative way of creating a path request
			//ABPath p = ABPath.Construct(currentPosition, targetPoint, null);
			//seeker.StartPath(p);

			// Create a new path request
			// The OnPathComplete method will later be called with the result
			seeker.StartPath(currentPosition, destination);
		}

		
		public virtual void OnTargetReached () {
		}

		
		protected virtual void OnPathComplete (Path _p) {
			ABPath p = _p as ABPath;

			if (p == null) throw new System.Exception("This function only handles ABPaths, do not use special path types");

			canSearchAgain = true;

			// Increase the reference count on the path.
			// This is used for path pooling
			p.Claim(this);

			// Path couldn't be calculated of some reason.
			// More info in p.errorLog (debug string)
			if (p.error) {
				p.Release(this);
				return;
			}

			if (interpolatePathSwitches) {
				ConfigurePathSwitchInterpolation();
			}


			// Replace the old path
			var oldPath = path;
			path = p;
			reachedEndOfPath = false;

			
			if (path.vectorPath != null && path.vectorPath.Count == 1) {
				path.vectorPath.Insert(0, GetFeetPosition());
			}

			// Reset some variables
			ConfigureNewPath();

			
			if (oldPath != null) oldPath.Release(this);

			if (interpolator.remainingDistance < 0.0001f && !reachedEndOfPath) {
				reachedEndOfPath = true;
				OnTargetReached();
			}
		}

		public void SetPath (Path path) {
			if (path.PipelineState == PathState.Created) {
				lastRepath = Time.time;
				canSearchAgain = false;
				seeker.CancelCurrentPathRequest();
				seeker.StartPath(path);
			} else if (path.PipelineState == PathState.Returned) {

				if (seeker.GetCurrentPath() != path) seeker.CancelCurrentPathRequest();
				else throw new System.ArgumentException("If you calculate the path using seeker.StartPath then this script will pick up the calculated path anyway as it listens for all paths the Seeker finishes calculating. You should not call SetPath in that case.");

				OnPathComplete(path);
			} else {
				throw new System.ArgumentException("You must call the SetPath method with a path that either has been completely calculated or one whose path calculation has not been started at all. It looks like the path calculation for the path you tried to use has been started, but is not yet finished.");
			}
		}

		protected virtual void ConfigurePathSwitchInterpolation () {
			bool reachedEndOfPreviousPath = interpolator.valid && interpolator.remainingDistance < 0.0001f;

			if (interpolator.valid && !reachedEndOfPreviousPath) {
				previousMovementOrigin = interpolator.position;
				previousMovementDirection = interpolator.tangent.normalized * interpolator.remainingDistance;
				pathSwitchInterpolationTime = 0;
			} else {
				previousMovementOrigin = Vector3.zero;
				previousMovementDirection = Vector3.zero;
				pathSwitchInterpolationTime = float.PositiveInfinity;
			}
		}

		public virtual Vector3 GetFeetPosition () {
			return position;
		}

		protected virtual void ConfigureNewPath () {
			var hadValidPath = interpolator.valid;
			var prevTangent = hadValidPath ? interpolator.tangent : Vector3.zero;

			interpolator.SetPath(path.vectorPath);
			interpolator.MoveToClosestPoint(GetFeetPosition());

			if (interpolatePathSwitches && switchPathInterpolationSpeed > 0.01f && hadValidPath) {
				var correctionFactor = Mathf.Max(-Vector3.Dot(prevTangent.normalized, interpolator.tangent.normalized), 0);
				interpolator.distance -= speed*correctionFactor*(1f/switchPathInterpolationSpeed);
			}
		}

		protected virtual void Update () {
			if (shouldRecalculatePath) SearchPath();
			if (canMove) {
				Vector3 nextPosition;
				Quaternion nextRotation;
				MovementUpdate(Time.deltaTime, out nextPosition, out nextRotation);
				FinalizeMovement(nextPosition, nextRotation);
			}
		}

		public void MovementUpdate (float deltaTime, out Vector3 nextPosition, out Quaternion nextRotation) {
			if (updatePosition) simulatedPosition = tr.position;
			if (updateRotation) simulatedRotation = tr.rotation;

			Vector3 direction;

			nextPosition = CalculateNextPosition(out direction, isStopped ? 0f : deltaTime);

			if (enableRotation) nextRotation = SimulateRotationTowards(direction, deltaTime);
			else nextRotation = simulatedRotation;
		}

		public void FinalizeMovement (Vector3 nextPosition, Quaternion nextRotation) {
			previousPosition2 = previousPosition1;
			previousPosition1 = simulatedPosition = nextPosition;
			simulatedRotation = nextRotation;
			if (updatePosition) tr.position = nextPosition;
			if (updateRotation) tr.rotation = nextRotation;
		}

		Quaternion SimulateRotationTowards (Vector3 direction, float deltaTime) {
			// Rotate unless we are really close to the target
			if (direction != Vector3.zero) {
				Quaternion targetRotation = Quaternion.LookRotation(direction, orientation == OrientationMode.YAxisForward ? Vector3.back : Vector3.up);
				if (orientation == OrientationMode.YAxisForward) targetRotation *= Quaternion.Euler(90, 0, 0);
				return Quaternion.Slerp(simulatedRotation, targetRotation, deltaTime * rotationSpeed);
			}
			return simulatedRotation;
		}

		protected virtual Vector3 CalculateNextPosition (out Vector3 direction, float deltaTime) {
			if (!interpolator.valid) {
				direction = Vector3.zero;
				return simulatedPosition;
			}

			interpolator.distance += deltaTime * speed;

			if (interpolator.remainingDistance < 0.0001f && !reachedEndOfPath) {
				reachedEndOfPath = true;
				OnTargetReached();
			}

			direction = interpolator.tangent;
			pathSwitchInterpolationTime += deltaTime;
			var alpha = switchPathInterpolationSpeed * pathSwitchInterpolationTime;
			if (interpolatePathSwitches && alpha < 1f) {
				
				Vector3 positionAlongPreviousPath = previousMovementOrigin + Vector3.ClampMagnitude(previousMovementDirection, speed * pathSwitchInterpolationTime);

				
				return Vector3.Lerp(positionAlongPreviousPath, interpolator.position, alpha);
			} else {
				return interpolator.position;
			}
		}

		protected override int OnUpgradeSerializedData (int version, bool unityThread) {
			#pragma warning disable 618
			if (unityThread && targetCompatibility != null) target = targetCompatibility;
			#pragma warning restore 618
			return 2;
		}
	}
}
